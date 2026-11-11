package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * 自己算功率的 shooter 速度控制（不用 SDK 自带的 RUN_USING_ENCODER PIDF）。
 *
 * power = 电压补偿 × (kS + kV·target)          ← 前馈：负责"大概给多少功率"
 *       + kP·error + I + kD·(-dVel/dt)           ← 反馈：只修正剩下的误差
 *       （误差太大时直接 boost 满功率，快速回速）
 *
 * 调参顺序：
 *   1. kP = kI = kD = 0，boost 关掉。调 kV 让稳态速度≈目标（kS 一般 0.02~0.06）。
 *      不同目标速度都差不多准，前馈就对了。
 *   2. 加 kP，直到回速快但不振荡。
 *   3. 还有一点稳态误差再加一点 kI。kD 通常保持 0。
 *   4. 打开 boost，调 boostThreshold，让发射掉速后尽快回来。
 *
 * 所有参数在 Panels 里实时改，立即生效。
 * 手柄：右扳机 = 目标速度 / 松开 = 空转速度；A = intake；右肩键 = 开门；
 *       dpad_up/down 切换 pollen / nectar / 两个一起。
 */
@TeleOp(name = "Test Shooter FF+PID", group = "test")
@Configurable
public class ShooterFFPIDTest extends LinearOpMode {
    // ---------- 目标 ----------
    public static double shootVelocity = 1820;   // ticks/s
    public static double idleVelocity = 600;     // ticks/s

    // ---------- 前馈（按 12V 标定） ----------
    public static double kS = 0.08;      // 克服静摩擦的功率
    public static double kV = 0.0005;    // 功率 / (ticks/s)，≈ 1 / 12V 时的满速
    public static double nominalVoltage = 12.0;

    // ---------- 反馈 ----------
    public static double kP = 0.018;     // 功率 / (ticks/s)
    public static double kI = 0.0005;    // 功率 / ticks
    public static double kD = 0.0;       // 功率 / (ticks/s²)，作用在测量值上，切换目标不会尖峰
    public static double iZone = 150;    // 误差小于这个才积分（ticks/s）
    public static double iMax = 0.15;    // 积分项最大功率

    // ---------- boost：掉速很多时满功率 ----------
    public static boolean useBoost = true;
    public static double boostThreshold = 200; // 低于目标多少 ticks/s 开始满功率

    // ---------- 测量 ----------
    public static double velocityFilterAlpha = 0.6; // 1 = 不滤波；越小越平滑但越滞后
    public static double readyTolerance = 40;       // 误差在这范围内算"到速"

    // 0 = pollen, 1 = nectar, 2 = both
    public static int shooterSelect = 2;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        DcMotorEx pollen = hardwareMap.get(DcMotorEx.class, "PollenShooter");
        DcMotorEx nectar = hardwareMap.get(DcMotorEx.class, "NectarShooter");
        Servo pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        Servo nectarGate = hardwareMap.get(Servo.class, "nectarGate");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        for (DcMotorEx m : new DcMotorEx[]{pollen, nectar}) {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // 编码器照样能读速度，但功率由我们自己算
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        FlywheelController pollenCtrl = new FlywheelController();
        FlywheelController nectarCtrl = new FlywheelController();

        telemetry.addLine("FF+PID shooter test ready");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        double filteredVoltage = battery.getVoltage();
        boolean lastUp = false, lastDown = false;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            double dt = Math.max(loopTimer.seconds(), 1e-3);
            loopTimer.reset();

            // 电压读一次很慢且有噪声，做个慢滤波
            filteredVoltage += 0.05 * (battery.getVoltage() - filteredVoltage);

            if (gamepad1.dpad_up && !lastUp) shooterSelect = (shooterSelect + 1) % 3;
            if (gamepad1.dpad_down && !lastDown) shooterSelect = (shooterSelect + 2) % 3;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            double target = gamepad1.right_trigger > 0.5 ? shootVelocity : idleVelocity;
            boolean runPollen = shooterSelect != 1;
            boolean runNectar = shooterSelect != 0;

            double pollenPower = pollenCtrl.update(runPollen ? target : 0, pollen.getVelocity(), filteredVoltage, dt);
            double nectarPower = nectarCtrl.update(runNectar ? target : 0, nectar.getVelocity(), filteredVoltage, dt);
            pollen.setPower(pollenPower);
            nectar.setPower(nectarPower);

            double intakePower = gamepad1.a ? 1 : 0;
            intake.setPower(intakePower);
            intake1.setPower(intakePower);

            if (gamepad1.right_bumper) {
                nectarGate.setPosition(0);
                pollenGate.setPosition(0.9);
            } else {
                nectarGate.setPosition(0.33);
                pollenGate.setPosition(0.74);
            }

            panelsTelemetry.addData("Shooter Select", shooterSelect == 0 ? "pollen" : shooterSelect == 1 ? "nectar" : "both");
            panelsTelemetry.addData("Shooter Target Velocity", target);
            pollenCtrl.addTelemetry(panelsTelemetry, "Pollen");
            nectarCtrl.addTelemetry(panelsTelemetry, "Nectar");
            panelsTelemetry.addData("Battery Voltage", filteredVoltage);
            panelsTelemetry.addData("Loop ms", dt * 1000);
            panelsTelemetry.update(telemetry);
        }

        pollen.setPower(0);
        nectar.setPower(0);
    }

    /** 单个飞轮的 前馈 + PID 控制器。参数读外层的静态字段，所以 Panels 改了立刻生效。 */
    private static class FlywheelController {
        private double filteredVel = 0;
        private double lastFilteredVel = 0;
        private double integral = 0; // 已经乘过 kI，单位是功率
        private double lastTarget = Double.NaN;
        private boolean initialized = false;

        // 仅用于 telemetry
        private double target, error, ff, pTerm, dTerm, power;
        private boolean boosting;

        double update(double target, double measuredVel, double voltage, double dt) {
            this.target = target;

            if (!initialized) {
                filteredVel = lastFilteredVel = measuredVel;
                initialized = true;
            }
            double alpha = Range.clip(velocityFilterAlpha, 0.01, 1.0);
            lastFilteredVel = filteredVel;
            filteredVel += alpha * (measuredVel - filteredVel);

            if (target != lastTarget) {
                integral = 0;
                lastTarget = target;
            }

            if (target <= 0) {
                integral = 0;
                error = -filteredVel;
                ff = pTerm = dTerm = 0;
                boosting = false;
                power = 0;
                return 0; // 飞轮自己滑停，不反向刹车
            }

            error = target - filteredVel;
            double voltageComp = nominalVoltage / Math.max(voltage, 6.0);

            ff = (kS + kV * target) * voltageComp;
            pTerm = kP * error;
            // D 作用在测量值上：dError/dt = -dVel/dt（目标不变时）
            dTerm = -kD * (filteredVel - lastFilteredVel) / dt;

            boosting = useBoost && error > boostThreshold;

            if (boosting) {
                integral = 0;
                power = 1.0;
            } else {
                double unclipped = ff + pTerm + integral + dTerm;
                // 条件积分防饱和：误差够小，且没有在往饱和方向继续推时才积分
                boolean saturatedHigh = unclipped >= 1.0 && error > 0;
                boolean saturatedLow = unclipped <= 0.0 && error < 0;
                if (Math.abs(error) < iZone && !saturatedHigh && !saturatedLow) {
                    integral += kI * error * dt;
                    integral = Range.clip(integral, -iMax, iMax);
                }
                power = Range.clip(ff + pTerm + integral + dTerm, 0.0, 1.0);
            }
            return power;
        }

        void addTelemetry(TelemetryManager t, String name) {
            t.addData(name + " Velocity", filteredVel);
            t.addData(name + " Error", error);
            t.addData(name + " Ready", Math.abs(error) < readyTolerance && target > 0);
            t.addData(name + " Power", power);
            t.addData(name + " FF", ff);
            t.addData(name + " P", pTerm);
            t.addData(name + " I", integral);
            t.addData(name + " D", dTerm);
            t.addData(name + " Boost", boosting);
        }
    }
}
