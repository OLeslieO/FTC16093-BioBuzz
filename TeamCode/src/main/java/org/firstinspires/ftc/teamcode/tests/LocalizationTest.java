package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;


@TeleOp(name = "Localization Test", group = "test")
@Configurable
public class LocalizationTest extends LinearOpMode {

    public static double START_X = 55.64;
    public static double START_Y = 7.677;
    public static double START_HEADING_DEG = 90.0;

    /** 慢速模式下的底盘功率系数。 */
    public static double SLOW_SCALE = 0.35;

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Follower follower = Constants.create(hardwareMap);
        PPDriveSubsystem drive = new PPDriveSubsystem(follower, out);
        drive.setRobotCentric(true);
        drive.setPose(startPose());

        while (opModeInInit()) {
            follower.update();
            Pose p = drive.getPose();
            out.addLine("Localization Test");
            out.addData("Start", "(%.2f, %.2f, %.1f°)", START_X, START_Y, START_HEADING_DEG);
            out.addData("Pose", "(%.2f, %.2f, %.1f°)", p.x(), p.y(), Math.toDegrees(p.heading()));
            out.addLine("车摆在起点再按 START；A 切换驾驶模式，X 重置位姿");
            out.update();
        }
        if (isStopRequested()) {
            return;
        }

        drive.startTeleOp();

        boolean lastA = false;
        boolean lastX = false;

        try {
            while (opModeIsActive()) {
                // ---- 按键（上升沿）----
                if (gamepad1.a && !lastA) {
                    drive.toggleDriveMode();
                }
                if (gamepad1.x && !lastX) {
                    drive.setPose(startPose());
                }
                lastA = gamepad1.a;
                lastX = gamepad1.x;

                // ---- 底盘 ----
                double forward = -gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x; // PPDrive：+turn = 逆时针
                drive.setPowerScale(gamepad1.left_bumper ? SLOW_SCALE : 1.0);
                drive.drive(forward, strafe, turn);

                // 更新定位并输出 X / Y / Heading
                drive.update();

                Pose p = drive.getPose();
                Velocity v = drive.getVelocity();
                out.addData("Start", "(%.2f, %.2f, %.1f°)", START_X, START_Y, START_HEADING_DEG);
                out.addData("Dist From Start", "%.2f in",
                        Math.hypot(p.x() - START_X, p.y() - START_Y));
                out.addData("Velocity", "vx %.1f  vy %.1f in/s  ω %.1f°/s",
                        v.vx, v.vy, Math.toDegrees(v.omega));
                out.update();
            }
        } finally {
            drive.stop();
        }
    }

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }
}
