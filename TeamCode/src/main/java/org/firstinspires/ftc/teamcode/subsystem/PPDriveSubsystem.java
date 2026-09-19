package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;

@Config
public class PPDriveSubsystem {


    // Dashboard Configuration
    public static double headingKp = 1.75;
    public static double headingKi = 0.0;
    public static double headingKd = 0.09;

    public static double maxHeadingPower = 0.65;
    public static double joystickDeadband = 0.04;

    // Pose Transfer
    private static Pose transferredPose = Pose.zero();
    private static boolean hasTransferredPose = false;

    // Hardware / Framework
    private final Follower follower;
    private final Telemetry telemetry;
    private final PIDController headingController;


    private boolean teleOpEnabled = false;
    private boolean robotCentric = true;
    private boolean headingLocked = false;


    private double headingTargetRadians = 0.0;

    private double forwardInput = 0.0;
    private double strafeInput = 0.0;
    private double turnInput = 0.0;

    private double powerScale = 1.0;

    // Constructor
    public PPDriveSubsystem(
            Follower follower,
            Telemetry telemetry
    ) {
        this.follower = Objects.requireNonNull(follower);
        this.telemetry = Objects.requireNonNull(telemetry);

        headingController = new PIDController(
                headingKp,
                headingKi,
                headingKd
        );
    }

    // TeleOp

    /**
     * Enable manual TeleOp driving.
     */
    public void startTeleOp() {
        stop();

        teleOpEnabled = true;

        follower.manual(DrivePowers.zero());
    }

    /**
     * Disable manual TeleOp driving and stop the drivetrain.
     */
    public void stop() {
        teleOpEnabled = false;

        forwardInput = 0.0;
        strafeInput = 0.0;
        turnInput = 0.0;

        unlockHeading();

        follower.stop();
        follower.drivetrain.stop(true);
    }

    /**
     * Set driver inputs.
     *
     * forward:
     *     +1 = forward
     *
     * strafe:
     *     +1 = right
     *
     * turn:
     *     +1 = counter-clockwise
     */
    public void drive(
            double forward,
            double strafe,
            double turn
    ) {
        if (!teleOpEnabled) {
            return;
        }

        forwardInput = clip(forward, -1.0, 1.0);
        strafeInput = clip(strafe, -1.0, 1.0);
        turnInput = clip(turn, -1.0, 1.0);
    }

    // Drive Mode
    public void setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
    }

    public boolean isRobotCentric() {
        return robotCentric;
    }

    public void toggleDriveMode() {
        robotCentric = !robotCentric;
    }

    // Power
    public void setPowerScale(double powerScale) {
        this.powerScale = clip(
                powerScale,
                0.0,
                1.0
        );
    }

    public double getPowerScale() {
        return powerScale;
    }

    // Heading Lock
    public void lockHeading(double targetRadians) {
        headingTargetRadians =
                Angle.normalizeSigned(targetRadians);

        headingController.reset();
        headingLocked = true;
    }

    public void lockCurrentHeading() {
        lockHeading(follower.pose().heading());
    }

    public void unlockHeading() {
        headingLocked = false;
        headingController.reset();
    }

    public boolean isHeadingLocked() {
        return headingLocked;
    }

    public double getHeadingTargetRadians() {
        return headingTargetRadians;
    }


    // Pose / Localization
    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        Objects.requireNonNull(pose);

        follower.setPose(pose);
        headingController.reset();
    }

    public void setStartingPose(Pose pose) {
        setPose(pose);
    }

    /**
     * Save the current pose for transferring from
     * Autonomous to TeleOp.
     */
    public void savePose() {
        Pose pose = follower.pose();

        transferredPose = new Pose(
                pose.x(),
                pose.y(),
                pose.heading()
        );

        hasTransferredPose = true;
    }

    /**
     * Save a specific pose for the next OpMode.
     */
    public static void saveTransferredPose(Pose pose) {
        Objects.requireNonNull(pose);

        transferredPose = new Pose(
                pose.x(),
                pose.y(),
                pose.heading()
        );

        hasTransferredPose = true;
    }

    /**
     * Restore the previously saved pose.
     *
     * @return true if a pose was restored.
     */
    public boolean restorePose() {
        if (!hasTransferredPose) {
            return false;
        }

        setPose(transferredPose);
        return true;
    }

    public static boolean hasTransferredPose() {
        return hasTransferredPose;
    }

    public static Pose getTransferredPose() {
        return transferredPose;
    }

    public static void clearTransferredPose() {
        transferredPose = Pose.zero();
        hasTransferredPose = false;
    }

    // Pedro Update
    /**
     * Must be called once per OpMode loop.
     *
     * This is the only place in this subsystem where
     * follower.update() should be called.
     */
    public void update() {

        if (teleOpEnabled) {
            updateManualDrive();
        }

        follower.update();

        sendTelemetry();
    }

    /**
     * Ivy periodic command.
     *
     * Schedule this once when using Ivy.
     */
    public Command periodic() {
        return infinite(this::update);
    }

    // Manual Drive
    private void updateManualDrive() {

        headingController.kP = headingKp;
        headingController.kI = headingKi;
        headingController.kD = headingKd;

        double forward = shapeInput(forwardInput);
        double strafe = shapeInput(strafeInput);
        double turn = shapeInput(turnInput);

        DrivePowers powers;

        if (robotCentric) {

            powers = new DrivePowers(
                    forward,
                    strafe,
                    turn
            );

        } else {

            powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    follower.pose().heading()
            );
        }

        if (headingLocked) {

            powers = ManualDrive.headingLock(
                    follower,
                    headingController,
                    powers,
                    headingTargetRadians
            );

            double turnLimit =
                    clip(maxHeadingPower, 0.0, 1.0);

            powers = new DrivePowers(
                    powers.forward(),
                    powers.strafe(),
                    clip(
                            powers.turn(),
                            -turnLimit,
                            turnLimit
                    )
            );
        }

        /*
         * Normalize the three axes so the combined command
         * does not exceed the requested power scale.
         */
        double denominator = Math.max(
                1.0,
                Math.abs(powers.forward())
                        + Math.abs(powers.strafe())
                        + Math.abs(powers.turn())
        );

        double outputScale =
                powerScale / denominator;

        powers = new DrivePowers(
                powers.forward() * outputScale,
                powers.strafe() * outputScale,
                powers.turn() * outputScale
        );

        follower.manual(powers);
    }


    // Input Processing

    private static double shapeInput(double input) {

        double deadband = clip(
                joystickDeadband,
                0.0,
                0.99
        );

        if (Math.abs(input) <= deadband) {
            return 0.0;
        }

        double magnitude =
                (Math.abs(input) - deadband)
                        / (1.0 - deadband);

        return Math.copySign(
                magnitude * magnitude,
                input
        );
    }

    // Telemetry
    private void sendTelemetry() {

        Pose pose = follower.pose();

        telemetry.addData(
                "Drive Mode",
                follower.mode()
        );

        telemetry.addData(
                "Drive Type",
                robotCentric
                        ? "Robot Centric"
                        : "Field Centric"
        );

        telemetry.addData(
                "X",
                "%.2f",
                pose.x()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                pose.y()
        );

        telemetry.addData(
                "Heading",
                "%.1f°",
                Math.toDegrees(pose.heading())
        );

        telemetry.addData(
                "Heading Lock",
                headingLocked
        );

        if (headingLocked) {
            telemetry.addData(
                    "Target Heading",
                    "%.1f°",
                    Math.toDegrees(
                            headingTargetRadians
                    )
            );
        }

        telemetry.addData(
                "Power Scale",
                "%.2f",
                powerScale
        );
    }

    // Utility
    private static double clip(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}