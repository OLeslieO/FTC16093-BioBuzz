package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimConfig;

/**
 * 云台舵机模式（位置模式）测试。两个舵机同一个齿轮，Through Bore 接电机口只用来读角度。
 *
 * <p>INIT 前把云台摆正（编码器在 INIT 时清零 = 云台 0°）。
 *
 * <ul>
 *   <li>MODE 0 STOP：关 PWM，舵机松开，可以用手拨。</li>
 *   <li>MODE 1 RAW：两个舵机直接给 RAW_POS（0~1）。先用这个确认两个舵机不打架、方向对。</li>
 *   <li>MODE 2 ANGLE：云台转到 TARGET_DEG，按 TURRET_DEG_PER_POS / CENTER_POS 换算成舵机位置。</li>
 *   <li>MODE 3 STEP：在 STEP_A / STEP_B 之间来回，统计过冲、稳定时间、稳态误差。</li>
 *   <li>MODE 4 CALIBRATE：从 CAL_POS_MIN 到 CAL_POS_MAX 分 CAL_STEPS 步，每步停下读编码器，
 *       最小二乘拟合 角度 = k·pos + b，给出 TURRET_DEG_PER_POS 和 CENTER_POS 的推荐值。</li>
 * </ul>
 *
 * <p>手柄：A 把 MODE 4 拟合结果写入 AutoAimConfig（只在本次运行有效，满意后手动抄回 AutoAimConfig.java）。
 */
@TeleOp(name = "Turret Position Servo Test", group = "test")
@Configurable
public class TurretPositionServoTest extends LinearOpMode {

    // 硬件名、方向、PWM 范围、换算系数都用 AutoAimConfig 里的，和比赛代码一致。

    // ---- 模式 ----

    public static int MODE = 0;

    public static double RAW_POS = 0.5;
    public static double TARGET_DEG = 0.0;

    public static double STEP_A_DEG = -45.0;
    public static double STEP_B_DEG = 45.0;
    public static double STEP_PERIOD = 2.0;

    /** 指令角度的限速（°/s），0 = 不限速直接跳。舵机冲太猛、过冲大时打开。 */
    public static double MAX_VEL_DPS = 0.0;

    /** 用编码器修正稳态误差的积分增益（1/s），0 = 纯开环。有齿隙或比例不准时再开。 */
    public static double TRIM_KI = 0.0;
    public static double TRIM_MAX_DEG = 5.0;
    /** 误差小于这个才积分（避免运动过程中积分）。 */
    public static double TRIM_WINDOW_DEG = 3.0;

    public static double MIN_ANGLE_DEG = -110.0;
    public static double MAX_ANGLE_DEG = 110.0;

    /** STEP 统计用：|误差| 进入并保持在这个范围内算稳定。 */
    public static double SETTLE_TOL_DEG = 1.0;

    // ---- MODE 4 标定 ----

    public static double CAL_POS_MIN = 0.3;
    public static double CAL_POS_MAX = 0.7;
    public static int CAL_STEPS = 9;
    public static double CAL_SETTLE_S = 0.8;

    // ---- 内部状态 ----

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private ServoImplEx servo0;
    private ServoImplEx servo1;
    private DcMotorEx encoder;

    private boolean pwmEnabled = false;
    private double appliedPwmMin = Double.NaN;
    private double appliedPwmMax = Double.NaN;

    private double commandDeg = 0.0;
    private double trimDeg = 0.0;

    // STEP 统计
    private double stepTarget = Double.NaN;
    private double stepFrom = 0.0;
    private double stepStart = 0.0;
    private double stepOvershoot = 0.0;
    private double stepLastOutside = 0.0;
    private String lastStepResult = "-";

    // 标定
    private int calIndex = 0;
    private double calStepStart = 0.0;
    private final double[] calPos = new double[64];
    private final double[] calAngle = new double[64];
    private int calCount = 0;
    private boolean calDone = false;
    private double fitSlope = Double.NaN;
    private double fitIntercept = Double.NaN;
    private double fitMaxResidual = Double.NaN;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        servo0 = hardwareMap.get(ServoImplEx.class, AutoAimConfig.SERVO_NAME_0);
        servo1 = hardwareMap.get(ServoImplEx.class, AutoAimConfig.SERVO_NAME_1);

        encoder = hardwareMap.get(DcMotorEx.class, AutoAimConfig.ENCODER_NAME);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0.0);

        // INIT 期间不给 PWM，免得一上电就猛转
        servo0.setPwmDisable();
        servo1.setPwmDisable();
        pwmEnabled = false;

        while (opModeInInit()) {
            panels.addData("angle_deg", angleDeg());
            panels.addLine("云台摆正后 START。逆时针（俯视）转 angle_deg 应增加");
            panels.update(telemetry);
        }
        if (isStopRequested()) {
            return;
        }

        int previousMode = -1;
        double lastTime = getRuntime();
        double stepToggle = getRuntime();
        boolean stepAtB = false;
        boolean lastA = false;

        try {
            while (opModeIsActive()) {
                double now = getRuntime();
                double dt = clip(now - lastTime, 0.001, 0.05);
                lastTime = now;

                applyHardwareConfig();

                double angle = angleDeg();
                String state = "-";
                double pos = Double.NaN;
                double target = Double.NaN;

                if (MODE != previousMode) {
                    previousMode = MODE;
                    commandDeg = angle;   // 从当前位置开始，限速时不会先跳一下
                    trimDeg = 0.0;
                    stepTarget = Double.NaN;
                    stepToggle = now;
                    stepAtB = false;
                    if (MODE == 4) {
                        calIndex = 0;
                        calCount = 0;
                        calDone = false;
                        calStepStart = now;
                    }
                }

                if (MODE == 1) {
                    pos = RAW_POS;
                    state = "RAW";

                } else if (MODE == 2 || MODE == 3) {
                    if (MODE == 3) {
                        if (now - stepToggle >= STEP_PERIOD) {
                            stepAtB = !stepAtB;
                            stepToggle = now;
                        }
                        target = stepAtB ? STEP_B_DEG : STEP_A_DEG;
                    } else {
                        target = TARGET_DEG;
                    }
                    target = clip(target, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

                    // 限速
                    if (MAX_VEL_DPS > 0.0) {
                        commandDeg = moveTowards(commandDeg, target, MAX_VEL_DPS * dt);
                    } else {
                        commandDeg = target;
                    }

                    double error = target - angle;

                    // 编码器积分修正：只在指令到位且接近目标时积分
                    if (TRIM_KI > 0.0 && commandDeg == target
                            && Math.abs(error) < TRIM_WINDOW_DEG) {
                        trimDeg = clip(trimDeg + TRIM_KI * error * dt,
                                -TRIM_MAX_DEG, TRIM_MAX_DEG);
                    } else if (TRIM_KI <= 0.0) {
                        trimDeg = 0.0;
                    }

                    pos = angleToPos(commandDeg + trimDeg);
                    updateStepStats(target, angle, now);
                    state = MODE == 3 ? "STEP" : "ANGLE";

                } else if (MODE == 4) {
                    pos = runCalibration(angle, now);
                    state = calDone ? "CAL DONE" : "CAL " + (calIndex + 1) + "/" + CAL_STEPS;
                }

                // 输出
                if (Double.isNaN(pos)) {
                    setPwmEnabled(false);
                    state = MODE == 0 ? "STOP (PWM off)" : state;
                } else {
                    pos = clip(pos, 0.0, 1.0);
                    setPwmEnabled(true);
                    servo0.setPosition(pos);
                    servo1.setPosition(clip(pos + AutoAimConfig.SERVO_1_TRIM, 0.0, 1.0));
                }

                // A：采用标定结果
                boolean a = gamepad1.a;
                if (a && !lastA && calDone && !Double.isNaN(fitSlope)) {
                    AutoAimConfig.TURRET_DEG_PER_POS = fitSlope;
                    AutoAimConfig.CENTER_POS = -fitIntercept / fitSlope;
                }
                lastA = a;

                // 遥测
                panels.addData("MODE", MODE);
                panels.addData("STATE", state);
                panels.addData("angle_deg", angle);
                panels.addData("vel_dps", velocityDps());
                if (!Double.isNaN(target)) {
                    panels.addData("target_deg", target);
                    panels.addData("command_deg", commandDeg);
                    panels.addData("error_deg", target - angle);
                    panels.addData("trim_deg", trimDeg);
                    panels.addData("expected_pos_angle_deg", posToAngle(angleToPos(target)));
                }
                if (!Double.isNaN(pos)) {
                    panels.addData("servo_pos", pos);
                    panels.addData("pos_implied_angle_deg", posToAngle(pos));
                }
                if (MODE == 3) {
                    panels.addData("last_step", lastStepResult);
                }
                if (calDone) {
                    panels.addData("fit TURRET_DEG_PER_POS", fitSlope);
                    panels.addData("fit CENTER_POS", -fitIntercept / fitSlope);
                    panels.addData("fit max residual deg", fitMaxResidual);
                    panels.addLine("按 A 写入本次运行参数");
                }
                panels.addData("TURRET_DEG_PER_POS", AutoAimConfig.TURRET_DEG_PER_POS);
                panels.addData("CENTER_POS", AutoAimConfig.CENTER_POS);
                panels.addData("loop_ms", dt * 1000.0);
                panels.update(telemetry);
            }
        } finally {
            servo0.setPwmDisable();
            servo1.setPwmDisable();
        }
    }

    // ---- STEP 统计 ----

    private void updateStepStats(double target, double angle, double now) {
        if (target != stepTarget) {
            if (!Double.isNaN(stepTarget)) {
                finishStep(angle);
            }
            stepFrom = Double.isNaN(stepTarget) ? angle : stepTarget;
            stepTarget = target;
            stepStart = now;
            stepLastOutside = now;
            stepOvershoot = 0.0;
        }
        double dir = Math.signum(stepTarget - stepFrom);
        if (dir != 0.0) {
            stepOvershoot = Math.max(stepOvershoot, (angle - stepTarget) * dir);
        }
        if (Math.abs(stepTarget - angle) > SETTLE_TOL_DEG) {
            stepLastOutside = now;
        }
    }

    private void finishStep(double angle) {
        double settle = stepLastOutside - stepStart;
        lastStepResult = String.format("%.0f→%.0f  overshoot %.2f°  settle %.2fs  final err %.2f°",
                stepFrom, stepTarget, stepOvershoot, settle, stepTarget - angle);
    }

    // ---- 标定 ----

    private double runCalibration(double angle, double now) {
        int steps = Math.max(2, Math.min(CAL_STEPS, calPos.length));
        if (calDone) {
            return posAtCalStep(steps / 2, steps);   // 结束后停在中间
        }
        double pos = posAtCalStep(calIndex, steps);
        if (now - calStepStart >= CAL_SETTLE_S) {
            calPos[calCount] = pos;
            calAngle[calCount] = angle;
            calCount++;
            calIndex++;
            calStepStart = now;
            if (calIndex >= steps) {
                fitLine();
                calDone = true;
                return posAtCalStep(steps / 2, steps);
            }
            pos = posAtCalStep(calIndex, steps);
        }
        return pos;
    }

    private static double posAtCalStep(int i, int steps) {
        return CAL_POS_MIN + (CAL_POS_MAX - CAL_POS_MIN) * i / (steps - 1);
    }

    private void fitLine() {
        double n = calCount, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < calCount; i++) {
            sx += calPos[i];
            sy += calAngle[i];
            sxx += calPos[i] * calPos[i];
            sxy += calPos[i] * calAngle[i];
        }
        double denom = n * sxx - sx * sx;
        if (Math.abs(denom) < 1e-9) {
            fitSlope = Double.NaN;
            return;
        }
        fitSlope = (n * sxy - sx * sy) / denom;
        fitIntercept = (sy - fitSlope * sx) / n;
        fitMaxResidual = 0.0;
        for (int i = 0; i < calCount; i++) {
            double r = calAngle[i] - (fitSlope * calPos[i] + fitIntercept);
            fitMaxResidual = Math.max(fitMaxResidual, Math.abs(r));
        }
    }

    // ---- 换算 ----

    private static double angleToPos(double deg) {
        return AutoAimConfig.CENTER_POS + deg / AutoAimConfig.TURRET_DEG_PER_POS;
    }

    private static double posToAngle(double pos) {
        return (pos - AutoAimConfig.CENTER_POS) * AutoAimConfig.TURRET_DEG_PER_POS;
    }

    // 硬件名、方向、PWM 范围、换算系数都用 AutoAimConfig 里的，和比赛代码一致。

    /** 只在参数变化时写硬件，避免每圈都发 PWM 设置命令拖慢循环。 */
    private void applyHardwareConfig() {
        if (AutoAimConfig.PWM_MIN_US != appliedPwmMin || AutoAimConfig.PWM_MAX_US != appliedPwmMax) {
            PwmControl.PwmRange range = new PwmControl.PwmRange(AutoAimConfig.PWM_MIN_US, AutoAimConfig.PWM_MAX_US);
            servo0.setPwmRange(range);
            servo1.setPwmRange(range);
            appliedPwmMin = AutoAimConfig.PWM_MIN_US;
            appliedPwmMax = AutoAimConfig.PWM_MAX_US;
        }
        servo0.setDirection(AutoAimConfig.SERVO_REVERSED_0 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo1.setDirection(AutoAimConfig.SERVO_REVERSED_1 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    private void setPwmEnabled(boolean on) {
        if (on == pwmEnabled) {
            return;
        }
        if (on) {
            servo0.setPwmEnable();
            servo1.setPwmEnable();
        } else {
            servo0.setPwmDisable();
            servo1.setPwmDisable();
        }
        pwmEnabled = on;
    }

    private double ticksPerTurretRev() {
        return AutoAimConfig.ENCODER_CPR / AutoAimConfig.ENCODER_TO_TURRET;
    }

    private double angleDeg() {
        double s = AutoAimConfig.ENCODER_REVERSED ? -1.0 : 1.0;
        return s * encoder.getCurrentPosition() / ticksPerTurretRev() * 360.0;
    }

    private double velocityDps() {
        double s = AutoAimConfig.ENCODER_REVERSED ? -1.0 : 1.0;
        return s * encoder.getVelocity() / ticksPerTurretRev() * 360.0;
    }

    private static double moveTowards(double current, double target, double maxDelta) {
        if (Math.abs(target - current) <= maxDelta) {
            return target;
        }
        return current + Math.copySign(maxDelta, target - current);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
