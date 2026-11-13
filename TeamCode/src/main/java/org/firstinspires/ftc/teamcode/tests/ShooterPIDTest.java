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
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * 纯 PID 的 shooter 速度控制（无前馈），两个飞轮用平均速度、同一个输出。
 *
 * error  = (target - avgVelocity) / 20
 * output = P·error + I·∫error·dt + D·dError/dt，限幅到 [-1, 1]
 * 目标切换时积分和 lastError 清零。
 *
 * 参数都在 Panels 里实时调。
 * 手柄：右扳机 = 目标速度 / 松开 = 空转速度；A = intake（transfer）；右肩键 = 开门。
 */
@TeleOp(name = "Test Shooter PID", group = "test")
@Configurable
public class ShooterPIDTest extends LinearOpMode {
    public static double shooterTargetVelocity = 1920;
    public static double shooterIdleVelocity = 1000;
    public static double shooterP = 0.12;
    public static double shooterI = 0.008;
    public static double shooterD = 0.0;
    public static double transferPower = 1.0;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private double integral;
    private double lastError;
    private double lastTargetVelocity = Double.NaN;

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

        for (DcMotorEx m : new DcMotorEx[]{pollen, nectar}) {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("PID shooter test ready");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            double targetVelocity = gamepad1.right_trigger > 0.5 ? shooterTargetVelocity : shooterIdleVelocity;
            resetOnTargetChange(targetVelocity);
            double pollenVelocity = pollen.getVelocity();
            double nectarVelocity = nectar.getVelocity();
            double currentVelocity = (pollenVelocity + nectarVelocity) / 2.0;
            double dt = Math.max(loopTimer.seconds(), 0.001);
            loopTimer.reset();

            double error = (targetVelocity - currentVelocity) / 20;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double output = shooterP * error
                    + shooterI * integral
                    + shooterD * derivative;
            double power = Range.clip(output, -1.0, 1.0);
            pollen.setPower(power);
            nectar.setPower(power);

            double intakePower = gamepad1.a ? transferPower : 0;
            intake.setPower(intakePower);
            intake1.setPower(intakePower);

            if (gamepad1.right_bumper) {
                nectarGate.setPosition(0);
                pollenGate.setPosition(0.5);
            } else {
                nectarGate.setPosition(0.33);
                pollenGate.setPosition(0.8);
            }

            panelsTelemetry.addData("Target Velocity", targetVelocity);
            panelsTelemetry.addData("Average Velocity", currentVelocity);
            panelsTelemetry.addData("Pollen Velocity", pollenVelocity);
            panelsTelemetry.addData("Nectar Velocity", nectarVelocity);
            panelsTelemetry.addData("Error (scaled)", error);
            panelsTelemetry.addData("Integral", integral);
            panelsTelemetry.addData("Derivative", derivative);
            panelsTelemetry.addData("Power", power);
            panelsTelemetry.addData("Loop ms", dt * 1000);
            panelsTelemetry.update(telemetry);
        }

        pollen.setPower(0);
        nectar.setPower(0);
        intake.setPower(0);
        intake1.setPower(0);
    }

    private void resetOnTargetChange(double targetVelocity) {
        if (targetVelocity == lastTargetVelocity) {
            return;
        }

        integral = 0;
        lastError = 0;
        lastTargetVelocity = targetVelocity;
    }
}
