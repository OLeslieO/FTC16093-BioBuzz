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

@TeleOp(name = "Test Shooter PIDF", group = "test")
@Configurable
public class ShooterPIDFTest extends LinearOpMode {
    // ticks/s
    public static double SHOOT_VEL = 1840;
    public static double IDLE_VEL = 1000;
    public static int shooterSelect = 0;
    public static boolean USE_PID = true;
    public static double VELOCITY_P = 0.006;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    public static boolean USE_F = true;
    public static double VELOCITY_F = 0.00049;
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

            double intakePower = gamepad1.a ? 0.9 : 0;
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
                measuredVel = motor.getVelocity();
                return;
            }
            measuredVel = motor.getVelocity();
            if (!usingPid) {
                motor.setVelocity(commandedVel);
                return;
            }
            if (usingF != USE_F) {
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
