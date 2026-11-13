 package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.function.DoubleSupplier;

/**
 * Nectar shooter 飞轮，自定义 PID + F 前馈速度控制（算法同 ShooterPIDFTest）。
 *
 * output = F·target + P·error + I·∫error·dt + D·dError/dt，限幅 [-1, 1]
 *   - 积分防饱和：只在没饱和、或误差在往回拉饱和时才积分；I = 0 时清空积分。
 *   - 换档、目标跳变超过 RESET_THRESHOLD、切换 USE_F / USE_PID 时重置控制器。
 *   - USE_PID = false 时改用 Hub 自带的 RUN_USING_ENCODER 速度控制。
 *
 * <p>所有参数都是 {@code public static}，可以在 Panels 里实时调，下一个循环生效。
 */
@Configurable
public class NectarShooterSubsystem extends SubsystemBase {

    public enum State {
        /** 加速。 */
        ACCELERATE,
        /** 空转。 */
        IDLE,
        /** 停止。 */
        STOP
    }

    public static String NECTAR_SHOOTER_NAME = "NectarShooter";

    // ---------- 目标（ticks/s） ----------
    public static double ACCELERATE_VEL = 1920;
    public static double IDLE_VEL = 600.0;
    public static double STOP_VEL = 0.0;

    // ---------- 按距离查表的射速（移动射球用） ----------
    /** 打开后 ACCELERATE 档按距离查表；关闭或没有距离来源时用固定的 ACCELERATE_VEL。 */
    public static boolean USE_DISTANCE_TABLE = true;
    /**
     * 距离（英寸，云台旋转轴到瞄准点 / 边走边射时的虚拟目标，必须递增）-> 射速（ticks/s）。
     * 实测：正对球门，y = 24 -> 1920，y = 8 -> 2000；瞄准点 y = 70.75，所以距离 = 70.75 - y。
     * 新测的点按距离从小到大插进去，两个数组长度要一样。
     */
    public static double[] SHOT_DIST_IN = {46.75, 62.75};
    public static double[] SHOT_VEL = {1920.0, 2020};
    /** 超出表的范围时，是否沿两端那一段的斜率外推（否则取端点值）。 */
    public static boolean EXTRAPOLATE = true;
    /** 查表结果的上下限（ticks/s），防止外推太离谱。 */
    public static double MIN_SHOT_VEL = 1880.0;
    public static double MAX_SHOT_VEL = 2200.0;

    // ---------- 控制器 ----------
    /** true = 自己算功率；false = Hub 自带速度控制。 */
    public static boolean USE_PID = true;
    /** 功率 / (ticks/s) 误差。 */
    public static double VELOCITY_P = 0.004;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    /** 前馈 = VELOCITY_F × 目标 ticks/s，起点 ≈ 1 / 满功率速度。 */
    public static boolean USE_F = true;
    public static double VELOCITY_F = 0.00036;
    /**
     * 按距离查表时目标每个循环都在小幅变化，只有换档或目标跳变超过这个值（ticks/s）
     * 才重置控制器，否则积分和 D 永远起不来。
     */
    public static double RESET_THRESHOLD = 150;
    /** 误差在这范围内算"到速"。 */
    public static double READY_TOLERANCE = 60;

    private final DcMotorEx nectarShooter;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private State state = State.ACCELERATE;
    private DoubleSupplier distanceSupplier = null;
    private double lastDistance = Double.NaN;

    private double commandedVel;
    private double measuredVel;
    private double integral;
    private double previousError;
    private boolean hasPreviousError;
    private boolean usingPid;
    private boolean usingF;
    private State lastState = null;

    // 仅用于 telemetry
    private double error, feedforward, pTerm, iTerm, dTerm, output;

    public NectarShooterSubsystem(HardwareMap hardwareMap) {
        nectarShooter = hardwareMap.get(DcMotorEx.class, NECTAR_SHOOTER_NAME);
        nectarShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        nectarShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        usingPid = USE_PID;
        usingF = USE_F;
        nectarShooter.setMode(runMode());
        stopMotor();
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void accelerate() {
        setState(State.ACCELERATE);
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void stop() {
        setState(State.STOP);
    }

    /**
     * 设置到目标的距离来源（英寸），例如 {@code aim::getShotDistance}。
     * 返回值 <= 0 视为无效，退回固定的 ACCELERATE_VEL。传 null 取消。
     */
    public void setDistanceSupplier(DoubleSupplier distanceSupplier) {
        this.distanceSupplier = distanceSupplier;
    }

    /** 最近一次用于查表的距离（英寸），没有时为 NaN。 */
    public double getDistance() {
        return lastDistance;
    }

    /** 按距离线性插值查射速表，再夹到 [MIN_SHOT_VEL, MAX_SHOT_VEL]。表无效时返回 ACCELERATE_VEL。 */
    public static double velocityForDistance(double distance) {
        double[] xs = SHOT_DIST_IN;
        double[] ys = SHOT_VEL;
        if (xs == null || ys == null || xs.length == 0 || xs.length != ys.length) {
            return ACCELERATE_VEL;
        }
        int last = xs.length - 1;
        double v;
        if (xs.length == 1) {
            v = ys[0];
        } else {
            if (!EXTRAPOLATE) {
                distance = Range.clip(distance, xs[0], xs[last]);
            }
            // 找所在的那一段；表外时用两端那一段外推
            int i = 0;
            while (i < last - 1 && distance > xs[i + 1]) {
                i++;
            }
            double span = xs[i + 1] - xs[i];
            double t = span > 0 ? (distance - xs[i]) / span : 1.0;
            v = ys[i] + t * (ys[i + 1] - ys[i]);
        }
        return Range.clip(v, MIN_SHOT_VEL, MAX_SHOT_VEL);
    }

    /** 当前档位对应的shooter速度 */
    public double getTargetVelocity() {
        switch (state) {
            case ACCELERATE:
                return accelerateVelocity();
            case IDLE:
                return IDLE_VEL;
            case STOP:
                return STOP_VEL;
            default:
                return ACCELERATE_VEL;
        }
    }

    private double accelerateVelocity() {
        lastDistance = Double.NaN;
        if (!USE_DISTANCE_TABLE || distanceSupplier == null) {
            return ACCELERATE_VEL;
        }
        double d = distanceSupplier.getAsDouble();
        if (!(d > 0) || Double.isInfinite(d)) {
            return ACCELERATE_VEL;
        }
        lastDistance = d;
        return velocityForDistance(d);
    }

    public double getVelocity() {
        return measuredVel;
    }

    public double getError() {
        return error;
    }

    /** 最近一次写给电机的功率（Hub 速度模式下为 0）。 */
    public double getPower() {
        return output;
    }

    public double getFeedforward() {
        return feedforward;
    }

    public double getPTerm() {
        return pTerm;
    }

    public double getITerm() {
        return iTerm;
    }

    public double getDTerm() {
        return dTerm;
    }

    /** 到速了没有，可以用来决定什么时候开门。 */
    public boolean isReady() {
        return commandedVel > 0 && Math.abs(commandedVel - measuredVel) < READY_TOLERANCE;
    }

    /** 每个循环算一次功率写给电机；加入 CommandScheduler 后自动调用。 */
    @Override
    public void periodic() {
        updateControlMode();
        measuredVel = nectarShooter.getVelocity();

        double target = getTargetVelocity();
        if (target <= 0) {
            if (commandedVel != 0 || lastState != state) {
                stopMotor(); // 飞轮自己滑停，不反向刹车
            }
            lastState = state;
            error = -measuredVel;
            return;
        }

        if (state != lastState || commandedVel == 0
                || Math.abs(target - commandedVel) > RESET_THRESHOLD) {
            resetController();
        }
        lastState = state;
        commandedVel = target;

        if (!usingPid) {
            error = commandedVel - measuredVel;
            nectarShooter.setVelocity(commandedVel);
            return;
        }
        if (usingF != USE_F) {
            // 积分是按旧前馈积出来的，重新开始
            usingF = USE_F;
            resetController();
        }

        double dt = Math.max(loopTimer.seconds(), 0.001);
        loopTimer.reset();
        error = commandedVel - measuredVel;
        double slope = hasPreviousError ? (error - previousError) / dt : 0.0;
        feedforward = usingF ? VELOCITY_F * commandedVel : 0.0;
        pTerm = VELOCITY_P * error;
        dTerm = VELOCITY_D * slope;
        double base = feedforward + pTerm + dTerm;
        double candidateIntegral = integral + error * dt;
        double trial = base + VELOCITY_I * candidateIntegral;

        // 只在没饱和、或误差在把输出往回拉时才积分
        if ((trial >= -1.0 && trial <= 1.0)
                || (trial > 1.0 && error < 0.0)
                || (trial < -1.0 && error > 0.0)) {
            integral = candidateIntegral;
        }
        if (VELOCITY_I == 0.0) {
            integral = 0.0;
        }
        iTerm = VELOCITY_I * integral;
        output = Range.clip(base + iTerm, -1.0, 1.0);
        previousError = error;
        hasPreviousError = true;

        nectarShooter.setPower(output);
    }

    private DcMotor.RunMode runMode() {
        return usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER;
    }

    private void stopMotor() {
        if (usingPid) {
            nectarShooter.setPower(0.0);
        } else {
            nectarShooter.setVelocity(0.0);
        }
        commandedVel = 0.0;
        feedforward = pTerm = iTerm = dTerm = output = 0.0;
        resetController();
    }

    private void updateControlMode() {
        if (usingPid == USE_PID) {
            return;
        }
        // 先用旧模式停下再切换（包括 Panels 里实时改）
        stopMotor();
        usingPid = USE_PID;
        nectarShooter.setMode(runMode());
        resetController();
    }

    private void resetController() {
        integral = 0.0;
        previousError = 0.0;
        hasPreviousError = false;
        loopTimer.reset();
    }
}
