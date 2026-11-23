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

import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

import java.util.List;

@TeleOp(name = "Test Shooter PIDF", group = "test")
@Configurable
public class ShooterPIDFTest extends LinearOpMode {
    // ticks/s
    public static double SHOOT_VEL = 1760;
    public static double IDLE_VEL = 1000;
    public static boolean USE_PID = true;
    public static double VELOCITY_P = 0.006;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    public static boolean USE_F = true;
    public static double VELOCITY_F = 0.0004;
    public static double AT_SPEED_TOLERANCE = 60;
    public static double READY_TO_SHOOT_TOLERANCE = 60;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 两个电机同时用、方向相反，一套 PIDF 参数同时作用在两个电机上
        FlywheelController shooter = new FlywheelController(
                hardwareMap.get(DcMotorEx.class, ShooterSubsystem.SHOOTER_UP_NAME),
                hardwareMap.get(DcMotorEx.class, ShooterSubsystem.SHOOTER_DOWN_NAME));
        Servo gate = hardwareMap.get(Servo.class, GateSubsystem.GATE_NAME);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("PIDF shooter test ready");
        telemetry.update();
        waitForStart();
        shooter.resetController();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            double target = gamepad1.right_trigger > 0.5 ? SHOOT_VEL : IDLE_VEL;
            shooter.setVelocity(target);
            shooter.periodic();

            double intakePower = gamepad1.a ? 1 : 0;
            intake.setPower(intakePower);
            preShooter.setPower(intakePower);

            gate.setPosition(gamepad1.right_bumper
                    ? GateSubsystem.SHOOT_POSITION
                    : GateSubsystem.STOP_POSITION);

            panelsTelemetry.addData("Mode", USE_PID ? (USE_F ? "PID+F" : "PID") : "Hub velocity");
            panelsTelemetry.addData("Target Velocity", target);
            shooter.addTelemetry(panelsTelemetry, "Shooter");
            panelsTelemetry.update(telemetry);
        }

        shooter.stop();
        intake.setPower(0);
        preShooter.setPower(0);
    }

    private static class FlywheelController {
        private final DcMotorEx up;
        private final DcMotorEx down;
        private final ElapsedTime loopTimer = new ElapsedTime();
        private double commandedVel;
        private double measuredVel, upVel, downVel;
        private double integral;
        private double previousError;
        private boolean hasPreviousError;
        private boolean usingPid;
        private boolean usingF;
        private double error, feedforward, pTerm, iTerm, dTerm, output;

        FlywheelController(DcMotorEx up, DcMotorEx down) {
            this.up = up;
            this.down = down;
            usingPid = USE_PID;
            usingF = USE_F;
            up.setDirection(DcMotorSimple.Direction.FORWARD);
            down.setDirection(DcMotorSimple.Direction.FORWARD);
            for (DcMotorEx m : new DcMotorEx[]{up, down}) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(runMode());
            }
            stop();
        }

        private void readVelocity() {
            upVel = up.getVelocity();
            downVel = down.getVelocity();
            measuredVel = (upVel + downVel) / 2.0;
        }

        private void setPower(double power) {
            up.setPower(power);
            down.setPower(power);
        }

        private void setMotorVelocity(double velocity) {
            up.setVelocity(velocity);
            down.setVelocity(velocity);
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
                setMotorVelocity(velocity);
            }
        }

        void periodic() {
            updateControlMode();
            readVelocity();
            if (commandedVel == 0.0) {
                return;
            }
            if (!usingPid) {
                setMotorVelocity(commandedVel);
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

            setPower(output);
        }

        void stop() {
            if (usingPid) {
                setPower(0.0);
            } else {
                setMotorVelocity(0.0);
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
                setPower(0.0);
            } else {
                setMotorVelocity(0.0);
            }
            usingPid = USE_PID;
            up.setMode(runMode());
            down.setMode(runMode());
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
            t.addData(name + " Up Velocity", upVel);
            t.addData(name + " Down Velocity", downVel);
            t.addData(name + " Error", error);
            t.addData(name + " FF", feedforward);
            t.addData(name + " P", pTerm);
            t.addData(name + " I", iTerm);
            t.addData(name + " D", dTerm);
            t.addData(name + " Output", output);
            t.addData(name + " Up Power", up.getPower());
            t.addData(name + " Down Power", down.getPower());
            t.addData(name + " At Speed", isAtSpeed());
            t.addData(name + " Ready", isReadyToShoot());
        }
    }
}
