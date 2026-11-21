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

import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

import java.util.List;


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

        DcMotorEx shooterUp = hardwareMap.get(DcMotorEx.class, ShooterSubsystem.SHOOTER_UP_NAME);
        DcMotorEx shooterDown = hardwareMap.get(DcMotorEx.class, ShooterSubsystem.SHOOTER_DOWN_NAME);
        Servo gate = hardwareMap.get(Servo.class, GateSubsystem.GATE_NAME);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");

        // 两个电机同时用、方向相反
        shooterUp.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterDown.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{shooterUp, shooterDown}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("PID shooter test ready");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            double targetVelocity = gamepad1.right_trigger > 0.5 ? shooterTargetVelocity : shooterIdleVelocity;
            resetOnTargetChange(targetVelocity);
            double upVelocity = shooterUp.getVelocity();
            double downVelocity = shooterDown.getVelocity();
            double currentVelocity = (upVelocity + downVelocity) / 2.0;
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
            shooterUp.setPower(power);
            shooterDown.setPower(power);

            double intakePower = gamepad1.a ? transferPower : 0;
            intake.setPower(intakePower);
            preShooter.setPower(intakePower);

            gate.setPosition(gamepad1.right_bumper
                    ? GateSubsystem.SHOOT_POSITION
                    : GateSubsystem.STOP_POSITION);

            panelsTelemetry.addData("Target Velocity", targetVelocity);
            panelsTelemetry.addData("Average Velocity", currentVelocity);
            panelsTelemetry.addData("Up Velocity", upVelocity);
            panelsTelemetry.addData("Down Velocity", downVelocity);
            panelsTelemetry.addData("Error (scaled)", error);
            panelsTelemetry.addData("Integral", integral);
            panelsTelemetry.addData("Derivative", derivative);
            panelsTelemetry.addData("Power", power);
            panelsTelemetry.addData("Loop ms", dt * 1000);
            panelsTelemetry.update(telemetry);
        }

        shooterUp.setPower(0);
        shooterDown.setPower(0);
        intake.setPower(0);
        preShooter.setPower(0);
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
