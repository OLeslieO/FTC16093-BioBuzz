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


@Configurable
public class ShooterSubsystem extends SubsystemBase {

    public enum State {
        ACCELERATE,
        IDLE,
        STOP,
        CUSTOM
    }

    public static String SHOOTER_UP_NAME = "shooterUp";
    public static String SHOOTER_DOWN_NAME = "shooterDown";

    public static double ACCELERATE_VEL = 1160;
    public static double IDLE_VEL = 800.0;
    public static double STOP_VEL = 0.0;

    public static boolean USE_DISTANCE_TABLE = true;
    /**
     * PIDF: p=0.0064, i=0, d=0, f=0.00048：
     *   (55.6, 22.76) -> 1090，(55.43, 14.76) -> 1110，(32.17, 20.28) -> 1150，
     *   (55.6, 6.7) -> 1160，(35.43, 9.06) -> 1180，(82.64, 8.07) -> 1210。
     * 距离 =  (RED_AIM_X, RED_AIM_Y) = (57.7, 53)：
     *   30.31，38.31，41.50，46.35，49.26，51.39。
     */
    public static double[] SHOT_DIST_IN = {30.31, 38.31, 41.50, 46.35, 49.26, 51.39};
    public static double[] SHOT_VEL = {1090.0, 1110.0, 1150.0, 1170.0, 1120.0, 1220.0};
    public static boolean EXTRAPOLATE = true;
    public static double MIN_SHOT_VEL = 1000.0;
    public static double MAX_SHOT_VEL = 1350.0;

    // ---------- 控制器 ----------
    public static boolean USE_PID = true;
    public static double VELOCITY_P = 0.0063;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    public static boolean USE_F = true;
    public static double VELOCITY_F = 0.00048;

    public static double RESET_THRESHOLD = 100;
    /** 误差在这范围内算"到速"。 */
    public static double READY_TOLERANCE = 60;

    private final DcMotorEx shooterUp;
    private final DcMotorEx shooterDown;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private State state = State.ACCELERATE;
    private double customVel = 0.0;
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

    private double error, feedforward, pTerm, iTerm, dTerm, output;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterUp = hardwareMap.get(DcMotorEx.class, SHOOTER_UP_NAME);
        shooterUp.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterDown = hardwareMap.get(DcMotorEx.class, SHOOTER_DOWN_NAME);
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        usingPid = USE_PID;
        usingF = USE_F;
        shooterUp.setMode(runMode());
        shooterDown.setMode(runMode());
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

    public void setVelocity(double velocity) {
        customVel = velocity;
        setState(State.CUSTOM);
    }


    public void setDistanceSupplier(DoubleSupplier distanceSupplier) {
        this.distanceSupplier = distanceSupplier;
    }

    /** 最近一次用于查表的距离*/
    public double getDistance() {
        return lastDistance;
    }

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

    public double getTargetVelocity() {
        switch (state) {
            case ACCELERATE:
                return accelerateVelocity();
            case IDLE:
                return IDLE_VEL;
            case STOP:
                return STOP_VEL;
            case CUSTOM:
                return customVel;
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

    public boolean isReady() {
        return commandedVel > 0 && Math.abs(commandedVel - measuredVel) < READY_TOLERANCE;
    }

    @Override
    public void periodic() {
        updateControlMode();
        measuredVel = shooterUp.getVelocity();

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
            shooterUp.setVelocity(commandedVel);
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

        shooterUp.setPower(output);
        shooterDown.setPower(output);
    }

    private DcMotor.RunMode runMode() {
        return usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER;
    }

    private void stopMotor() {
        if (usingPid) {
            shooterUp.setPower(0.0);
            shooterDown.setPower(0.0);
        } else {
            shooterUp.setVelocity(0.0);
            shooterDown.setVelocity(0.0);
        }
        commandedVel = 0.0;
        feedforward = pTerm = iTerm = dTerm = output = 0.0;
        resetController();
    }

    private void updateControlMode() {
        if (usingPid == USE_PID) {
            return;
        }
        stopMotor();
        usingPid = USE_PID;
        shooterUp.setMode(runMode());
        shooterDown.setMode(runMode());
        resetController();
    }

    private void resetController() {
        integral = 0.0;
        previousError = 0.0;
        hasPreviousError = false;
        loopTimer.reset();
    }
}
