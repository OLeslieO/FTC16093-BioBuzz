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
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;

import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

import java.util.List;

/**
 * Flywheel velocity test using SolversLib's SquIDFController:
 *   u = kP * sqrt(|e|) * sign(e) + kI * integral(e) + kD * de/dt + kF * setpoint
 *
 * Because the P term uses sqrt(error) (ticks/s), kP needs to be much larger than in the
 * linear PIDF test: e.g. 200 ticks/s error -> sqrt = ~14.
 *
 * Both shooter motors (shooterUp / shooterDown) run together in opposite directions,
 * driven by one controller on their average velocity with the same power.
 *
 * Controls (gamepad1):
 *   right trigger  - SHOOT_VEL (otherwise IDLE_VEL)
 *   left bumper    - toggle flywheels off/on
 *   a              - run intake
 *   right bumper   - open gates
 */
@TeleOp(name = "Test Shooter SquIDF", group = "test")
@Configurable
public class ShooterSquIDFTest extends LinearOpMode {
    // ticks/s
    public static double SHOOT_VEL = 1840;
    public static double IDLE_VEL = 1000;

    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00049;

    /** Bounds on the accumulated integral (ticks/s * s). Library default of +-1 is far too small for ticks/s. */
    public static double INTEGRAL_BOUND = 500;
    /** Multiplier applied to the integral when it has the opposite sign of the error (1 = no decay). */
    public static double INTEGRAL_DECAY = 0.5;
    /** Clear the integral when within AT_SPEED_TOLERANCE. */
    public static boolean CLEAR_I_AT_SP = false;

    /** Allow negative power (active braking) when above target. Off = coast down. */
    public static boolean ALLOW_REVERSE = false;

    public static double AT_SPEED_TOLERANCE = 60;
    public static double READY_TO_SHOOT_TOLERANCE = 60;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Flywheel shooter = new Flywheel(
                hardwareMap.get(DcMotorEx.class, ShooterSubsystem.SHOOTER_UP_NAME),
                hardwareMap.get(DcMotorEx.class, ShooterSubsystem.SHOOTER_DOWN_NAME));
        Servo gate = hardwareMap.get(Servo.class, GateSubsystem.GATE_NAME);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("SquIDF shooter test ready");
        telemetry.update();
        waitForStart();

        boolean lastLB = false;
        boolean enabled = true;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            if (gamepad1.left_bumper && !lastLB) enabled = !enabled;
            lastLB = gamepad1.left_bumper;

            double target = !enabled ? 0 : gamepad1.right_trigger > 0.5 ? SHOOT_VEL : IDLE_VEL;
            shooter.setTarget(target);
            shooter.update();

            double intakePower = gamepad1.a ? 0.9 : 0;
            intake.setPower(intakePower);
            intake1.setPower(intakePower);

            gate.setPosition(gamepad1.right_bumper
                    ? GateSubsystem.SHOOT_POSITION
                    : GateSubsystem.STOP_POSITION);

            panelsTelemetry.addData("Enabled", enabled);
            panelsTelemetry.addData("Target Velocity", target);
            shooter.addTelemetry(panelsTelemetry, "Shooter");
            panelsTelemetry.update(telemetry);
        }

        shooter.stop();
        intake.setPower(0);
        intake1.setPower(0);
    }

    private static class Flywheel {
        private final DcMotorEx up;
        private final DcMotorEx down;
        private final SquIDFController controller = new SquIDFController(kP, kI, kD, kF);
        private final PIDFController.IntegrationControl integration =
                new PIDFController.IntegrationControl(PIDFController.IntegrationBehavior.NONE, 1, -1, 1);
        private double target;
        private double measured, upVel, downVel;
        private double output;

        Flywheel(DcMotorEx up, DcMotorEx down) {
            this.up = up;
            this.down = down;
            // Opposite directions so both wheels spin the right way with the same power.
            up.setDirection(DcMotorSimple.Direction.REVERSE);
            down.setDirection(DcMotorSimple.Direction.FORWARD);
            for (DcMotorEx m : new DcMotorEx[]{up, down}) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            controller.setIntegrationControl(integration);
            setPower(0);
        }

        private void setPower(double power) {
            up.setPower(power);
            down.setPower(power);
        }

        void setTarget(double velocity) {
            if (velocity != target) {
                // Fresh integral / derivative history on every setpoint change.
                controller.reset();
            }
            target = velocity;
        }

        void update() {
            upVel = up.getVelocity();
            downVel = down.getVelocity();
            measured = (upVel + downVel) / 2.0;

            if (target == 0) {
                output = 0;
                setPower(0);
                return;
            }

            // Pull live Panels values every loop.
            controller.setPIDF(kP, kI, kD, kF);
            controller.setTolerance(AT_SPEED_TOLERANCE);
            integration.setIntegrationBounds(-INTEGRAL_BOUND, INTEGRAL_BOUND);
            integration.setDecayFactor(INTEGRAL_DECAY);
            integration.setIntegrationBehavior(CLEAR_I_AT_SP
                    ? PIDFController.IntegrationBehavior.CLEAR_AT_SP
                    : PIDFController.IntegrationBehavior.NONE);

            controller.setSetPoint(target);
            double raw = controller.calculate(measured);
            output = Math.max(ALLOW_REVERSE ? -1.0 : 0.0, Math.min(1.0, raw));
            setPower(output);
        }

        void stop() {
            target = 0;
            output = 0;
            setPower(0);
            controller.reset();
        }

        void addTelemetry(TelemetryManager t, String name) {
            double error = target - measured;
            t.addData(name + " Target", target);
            t.addData(name + " Velocity", measured);
            t.addData(name + " Up Velocity", upVel);
            t.addData(name + " Down Velocity", downVel);
            t.addData(name + " Error", error);
            t.addData(name + " P (sqrt)", kP * Math.sqrt(Math.abs(error)) * Math.signum(error));
            t.addData(name + " FF", kF * target);
            t.addData(name + " Output", output);
            t.addData(name + " Loop ms", controller.getPeriod() * 1000);
            t.addData(name + " At Speed", target != 0 && Math.abs(error) <= AT_SPEED_TOLERANCE);
            t.addData(name + " Ready", target > 0 && measured > target - READY_TO_SHOOT_TOLERANCE);
        }
    }
}
