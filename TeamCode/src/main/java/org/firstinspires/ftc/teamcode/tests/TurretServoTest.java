package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Turret Servo Test", group = "test")
@Configurable
public class TurretServoTest extends LinearOpMode {

    // ---- 硬件（INIT 时读取）----
    /** 两个舵机共同驱动云台，收到同一个功率指令。 */
    public static String SERVO_NAME_0 = "lala1";
    public static String SERVO_NAME_1 = "lala2";
    /** Through Bore 插在哪个电机端口，就填那个电机在配置里的名字。不会给这个端口输出功率。 */
    public static String ENCODER_NAME = "intake1";

    /** 每个舵机单独设方向（START 后实时生效）。两个舵机对着同一个齿轮装时，通常有一个要反转。 */
    public static boolean SERVO_REVERSED_0 = false;
    public static boolean SERVO_REVERSED_1 = false;
    public static boolean ENCODER_REVERSED = false;
    /** REV Through Bore v1：8192 counts / 编码器轴一圈。 */
    public static double ENCODER_CPR = 8192.0;
    /**
     * 编码器轴转一圈，云台转几圈。编码器装在云台轴上 = 1.0；
     * 装在舵机轴上 = 1 / 3.375。
     */
    public static double ENCODER_TO_TURRET = 0.14814815;

    // ---- 模式 ----
    public static int MODE = 0;
    public static double POWER = 0.1;
    public static double TARGET_DEG = 0.0;
    public static double STEP_A_DEG = -45.0;
    public static double STEP_B_DEG = 45.0;
    public static double STEP_PERIOD = 2.0;

    // ---- 限位（云台角度，相对正前方）----
    public static double MIN_ANGLE_DEG = -125.0;
    public static double MAX_ANGLE_DEG = 125.0;
    public static double LIMIT_MARGIN_DEG = 3.0;

    // ---- 控制参数（角度单位：度）----
    /** 位置环：速度指令 (°/s) = KP_POS × 误差 (°)。 */
    public static double KP_POS = 3;
    public static double DEADBAND_DEG = 0.5;
    /** 速度指令上限 (°/s)。Axon MAX 6V 空载约 520°/s，经 3.375 减速约 150°/s，留余量。 */
    public static double MAX_VEL_DPS = 130.0;
    /** 静摩擦 / CR 死区补偿（功率）。 */
    public static double kS = 0.13;
    /** 功率 / (°/s)。按满功率约 150°/s 估算的初值，需要实测。 */
    public static double kV = 0.006;
    /** 速度指令高于这个值才加 kS (°/s)，避免在目标附近来回抖。 */
    public static double KS_MIN_VEL_DPS = 2.0;
    /** 速度环 P：功率 / (°/s)。 */
    public static double KP_VEL = 0.0022;
    /** 速度测量低通（0~1，越大越跟手、噪声越大）。 */
    public static double VEL_FILTER_ALPHA = 0.5;
    public static double MAX_POWER = 1.0;

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private DcMotorEx encoder;
    private double ticksPerTurretRev;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CRServo[] servos = {
                hardwareMap.get(CRServo.class, SERVO_NAME_0),
                hardwareMap.get(CRServo.class, SERVO_NAME_1)
        };
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0.0);
        ticksPerTurretRev = ENCODER_CPR / ENCODER_TO_TURRET;

        while (opModeInInit()) {
            panels.addData("ticks / turret rev", ticksPerTurretRev);
            panels.addData("angle_deg", angleDeg());
            panels.addLine("云台摆正前方后再 INIT；用手转云台，向左转 angle_deg 应变大");
            panels.update(telemetry);
        }
        if (isStopRequested()) {
            return;
        }

        double filteredVel = 0.0;
        double lastTime = getRuntime();
        double stepStart = getRuntime();
        boolean stepAtB = false;

        try {
            while (opModeIsActive()) {
                double now = getRuntime();
                double dt = now - lastTime;
                lastTime = now;

                servos[0].setDirection(SERVO_REVERSED_0
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                servos[1].setDirection(SERVO_REVERSED_1
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

                double angle = angleDeg();
                filteredVel += VEL_FILTER_ALPHA * (velocityDps() - filteredVel);

                double target = Double.NaN;
                double velCmd = 0.0;
                double power;

                if (MODE == 1) {
                    power = POWER;
                } else if (MODE == 2 || MODE == 3) {
                    if (MODE == 3) {
                        if (now - stepStart >= STEP_PERIOD) {
                            stepAtB = !stepAtB;
                            stepStart = now;
                        }
                        target = stepAtB ? STEP_B_DEG : STEP_A_DEG;
                    } else {
                        target = TARGET_DEG;
                    }
                    target = clip(target, MIN_ANGLE_DEG + LIMIT_MARGIN_DEG,
                            MAX_ANGLE_DEG - LIMIT_MARGIN_DEG);

                    double error = target - angle;
                    velCmd = Math.abs(error) > DEADBAND_DEG ? KP_POS * error : 0.0;
                    velCmd = clip(velCmd, -MAX_VEL_DPS, MAX_VEL_DPS);

                    double ks = Math.abs(velCmd) > KS_MIN_VEL_DPS ? Math.copySign(kS, velCmd) : 0.0;
                    power = ks + kV * velCmd + KP_VEL * (velCmd - filteredVel);
                    panels.addData("error_deg", error);
                } else {
                    power = 0.0;
                }

                power = applyLimits(clip(power, -MAX_POWER, MAX_POWER), angle);
                for (CRServo servo : servos) {
                    servo.setPower(power);
                }

                if (!Double.isNaN(target)) {
                    panels.addData("target_deg", target);
                }
                panels.addData("MODE", MODE);
                panels.addData("angle_deg", angle);
                panels.addData("vel_dps", filteredVel);
                panels.addData("vel_cmd_dps", velCmd);
                panels.addData("power", power);
                panels.addData("encoder_ticks", rawTicks());
                panels.addData("loop_ms", dt * 1000.0);
                panels.update(telemetry);
            }
        } finally {
            for (CRServo servo : servos) {
                servo.setPower(0.0);
            }
        }
    }

    private int rawTicks() {
        return encoder.getCurrentPosition();
    }

    private double sign() {
        return ENCODER_REVERSED ? -1.0 : 1.0;
    }

    private double angleDeg() {
        return sign() * rawTicks() / ticksPerTurretRev * 360.0;
    }

    private double velocityDps() {
        return sign() * encoder.getVelocity() / ticksPerTurretRev * 360.0;
    }

    /** 贴近限位时不允许继续朝限位方向输出。 */
    private static double applyLimits(double power, double angleDeg) {
        if (power > 0.0 && angleDeg >= MAX_ANGLE_DEG - LIMIT_MARGIN_DEG) {
            return 0.0;
        }
        if (power < 0.0 && angleDeg <= MIN_ANGLE_DEG + LIMIT_MARGIN_DEG) {
            return 0.0;
        }
        return power;
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
