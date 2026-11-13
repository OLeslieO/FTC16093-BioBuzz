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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * 自定义 PID + F 前馈的 shooter 速度控制。pollen / nectar 各自一个电机带各自的飞轮，
 * 各用自己的编码器、各自一个控制器（参数共用）。
 *
 * output = F·target + P·error + I·∫error·dt + D·dError/dt，限幅 [-1, 1]
 *   - 积分防饱和：只在没饱和、或误差在往回拉饱和时才积分；I = 0 时清空积分。
 *   - 目标改变、切换 USE_F、切换 USE_PID 时重置控制器。
 *   - USE_PID = false 时改用 Hub 自带的 RUN_USING_ENCODER 速度控制。
 *
 * 所有参数在 Panels 里实时改。
 * 手柄：右扳机 = 射击速度 / 松开 = 空转速度（0 = 停）；A = intake；右肩键 = 开门；
 *       dpad_up/down 切换 pollen / nectar / 两个一起（也可以在 Panels 改 shooterSelect）。
 */
@TeleOp(name = "Test Shooter PIDF", group = "test")
@Configurable
public class ShooterPIDFTest extends LinearOpMode {
    // ticks/s
    public static double SHOOT_VEL = 1700;
    public static double IDLE_VEL = 1000;
    // 0 = pollen, 1 = nectar, 2 = both
    public static int shooterSelect = 0;
    // true = 自己算功率；false = Hub 自带速度控制
    public static boolean USE_PID = true;
    // 功率 / (ticks/s) 误差
    public static double VELOCITY_P = 0.0036;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    // 前馈 = VELOCITY_F × 目标 ticks/s，起点 ≈ 1 / 满功率速度
    public static boolean USE_F = true;
    public static double VELOCITY_F = 0.00052;
    public static double AT_SPEED_TOLERANCE = 60;
    public static double READY_TO_SHOOT_TOLERANCE = 60;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        FlywheelController pollen = new FlywheelController(hardwareMap.get(DcMotorEx.class, "PollenShooter"));
        FlywheelController nectar = new FlywheelController(hardwareMap.get(DcMotorEx.class, "NectarShooter"));
        Servo pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        Servo nectarGate = hardwareMap.get(Servo.class, "nectarGate");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("PIDF shooter test ready");
        telemetry.update();
        waitForStart();
        pollen.resetController();
        nectar.resetController();

        boolean lastUp = false, lastDown = false;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            if (gamepad1.dpad_up && !lastUp) shooterSelect = (shooterSelect + 1) % 3;
            if (gamepad1.dpad_down && !lastDown) shooterSelect = (shooterSelect + 2) % 3;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            int select = Math.floorMod(shooterSelect, 3);

            double target = gamepad1.right_trigger > 0.5 ? SHOOT_VEL : IDLE_VEL;
            pollen.setVelocity(select != 1 ? target : 0);
            nectar.setVelocity(select != 0 ? target : 0);
            pollen.periodic();
            nectar.periodic();

            double intakePower = gamepad1.a ? 1 : 0;
            intake.setPower(intakePower);
            intake1.setPower(intakePower);

            if (gamepad1.right_bumper) {
                nectarGate.setPosition(0);
                pollenGate.setPosition(0.5);
            } else {
                nectarGate.setPosition(0.33);
                pollenGate.setPosition(0.8);
            }

            panelsTelemetry.addData("Shooter Select", select == 0 ? "pollen" : select == 1 ? "nectar" : "both");
            panelsTelemetry.addData("Mode", USE_PID ? (USE_F ? "PID+F" : "PID") : "Hub velocity");
            panelsTelemetry.addData("Target Velocity", target);
            pollen.addTelemetry(panelsTelemetry, "Pollen");
            nectar.addTelemetry(panelsTelemetry, "Nectar");
            panelsTelemetry.update(telemetry);
        }

        pollen.stop();
        nectar.stop();
        intake.setPower(0);
        intake1.setPower(0);
    }

    /** 单个飞轮的 PID + F 控制器。参数读外层静态字段，Panels 改了立刻生效。 */
    private static class FlywheelController {
        private final DcMotorEx motor;
        private final ElapsedTime loopTimer = new ElapsedTime();
        private double commandedVel;
        private double measuredVel;
        private double integral;
        private double previousError;
        private boolean hasPreviousError;
        private boolean usingPid;
        private boolean usingF;
        // 仅 telemetry
        private double error, feedforward, pTerm, iTerm, dTerm, output;

        FlywheelController(DcMotorEx motor) {
            this.motor = motor;
            usingPid = USE_PID;
            usingF = USE_F;
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(runMode());
            stop();
        }

        private DcMotor.RunMode runMode() {
            return usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER;
        }

        void setVelocity(double velocity) {
            updateControlMode();
            if (velocity == 0.0) {
                if (commandedVel != 0.0) stop();
                return;
            }
            if (velocity != commandedVel) {
                resetController();
            }
            commandedVel = velocity;
            if (!usingPid) {
                motor.setVelocity(velocity);
            }
        }

        void periodic() {
            updateControlMode();
            if (commandedVel == 0.0) {
                // 停着的时候也显示真实速度（飞轮还在滑行）
                measuredVel = motor.getVelocity();
                return;
            }
            measuredVel = motor.getVelocity();
            if (!usingPid) {
                motor.setVelocity(commandedVel);
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
            output = Math.max(-1.0, Math.min(1.0, base + iTerm));
            previousError = error;
            hasPreviousError = true;

            motor.setPower(output);
        }

        void stop() {
            if (usingPid) {
                motor.setPower(0.0);
            } else {
                motor.setVelocity(0.0);
            }
            commandedVel = 0.0;
            error = feedforward = pTerm = iTerm = dTerm = output = 0.0;
            resetController();
        }

        private void updateControlMode() {
            if (usingPid == USE_PID) {
                return;
            }
            // 先用旧模式停下再切换（包括 Panels 里实时改）
            if (usingPid) {
                motor.setPower(0.0);
            } else {
                motor.setVelocity(0.0);
            }
            usingPid = USE_PID;
            motor.setMode(runMode());
            resetController();
        }

        void resetController() {
            integral = 0.0;
            previousError = 0.0;
            hasPreviousError = false;
            loopTimer.reset();
        }

        boolean isAtSpeed() {
            return commandedVel != 0.0 && Math.abs(commandedVel - measuredVel) <= AT_SPEED_TOLERANCE;
        }

        boolean isReadyToShoot() {
            return commandedVel > 0.0 && measuredVel > commandedVel - READY_TO_SHOOT_TOLERANCE;
        }

        void addTelemetry(TelemetryManager t, String name) {
            t.addData(name + " Commanded", commandedVel);
            t.addData(name + " Velocity", measuredVel);
            t.addData(name + " Error", error);
            t.addData(name + " FF", feedforward);
            t.addData(name + " P", pTerm);
            t.addData(name + " I", iTerm);
            t.addData(name + " D", dTerm);
            t.addData(name + " Output", output);
            t.addData(name + " Power", motor.getPower());
            t.addData(name + " At Speed", isAtSpeed());
            t.addData(name + " Ready", isReadyToShoot());
        }
    }
}
