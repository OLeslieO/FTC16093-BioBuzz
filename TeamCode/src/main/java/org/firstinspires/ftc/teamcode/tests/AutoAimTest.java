package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * 云台自动瞄准测试（红方）。
 *
 * <p><b>INIT 前：</b>车摆在 (START_X, START_Y)，车头朝 START_HEADING_DEG，
 * 云台手动摆到正前方（INIT 时会把这个位置当作云台零点）。
 *
 * <p>手柄 1：
 * <ul>
 *   <li>左摇杆：平移；右摇杆 X：转向；按住左肩键：慢速</li>
 *   <li>A：开 / 关自动瞄准</li>
 *   <li>Y：开 / 关边走边射</li>
 *   <li>X：把位姿重置回起点（车要先摆回起点）</li>
 * </ul>
 *
 * <p>telemetry 同时发到 Driver Station 和 Panels。
 */
@TeleOp(name = "AutoAim Test", group = "test")
@Configurable
public class AutoAimTest extends LinearOpMode {

    public static double START_X = 56.0;
    public static double START_Y = 8.0;
    public static double START_HEADING_DEG = 90.0;

    /** 慢速模式下的底盘功率系数。 */
    public static double SLOW_SCALE = 0.35;
    /** 三个摇杆都小于这个值时认为底盘在刹车，启用刹车预测。 */
    public static double BRAKE_STICK_THRESHOLD = 0.05;

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Alliance.set(Alliance.RED);

        Follower follower = Constants.create(hardwareMap);
        PPDriveSubsystem drive = new PPDriveSubsystem(follower, out);
        drive.setRobotCentric(true);
        drive.setPose(startPose());

        AutoAimSubsystem aim = new AutoAimSubsystem(
                hardwareMap, out, drive::getPose, drive::getVelocity, true);

        while (opModeInInit()) {
            follower.update();
            Pose p = drive.getPose();
            out.addLine("AutoAim Test — RED");
            out.addData("Start", "(%.1f, %.1f, %.0f°)", START_X, START_Y, START_HEADING_DEG);
            out.addData("Pose", "(%.1f, %.1f, %.1f°)", p.x(), p.y(), Math.toDegrees(p.heading()));
            out.addData("Aim Point", "(%.1f, %.1f)",
                    Alliance.get().getAimX(), Alliance.get().getAimY());
            out.addData("Turret Angle", "%.1f°", Math.toDegrees(aim.getTurretAngle()));
            out.addLine("云台要朝正前方；A 开瞄准，Y 边走边射，X 重置位姿");
            out.update();
        }
        if (isStopRequested()) {
            return;
        }

        drive.startTeleOp();

        boolean lastA = false;
        boolean lastY = false;
        boolean lastX = false;

        try {
            while (opModeIsActive()) {
                // ---- 按键（上升沿）----
                if (gamepad1.dpad_up && !lastA) {
                    if (aim.isEnabled()) {
                        aim.disable();
                    } else {
                        aim.enable();
                    }
                }
                if (gamepad1.y && !lastY) {
                    aim.setShootOnTheMove(!aim.isShootOnTheMove());
                }
                if (gamepad1.x && !lastX) {
                    drive.setPose(startPose());
                }
                lastA = gamepad1.dpad_up;
                lastY = gamepad1.y;
                lastX = gamepad1.x;

                // ---- 底盘 ----
                double forward = -gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x; // PPDrive：+turn = 逆时针
                drive.setPowerScale(gamepad1.left_bumper ? SLOW_SCALE : 1.0);
                drive.drive(forward, strafe, turn);

                aim.setBraking(Math.abs(forward) < BRAKE_STICK_THRESHOLD
                        && Math.abs(strafe) < BRAKE_STICK_THRESHOLD
                        && Math.abs(turn) < BRAKE_STICK_THRESHOLD);

                // 先更新定位，再用新位姿瞄准
                drive.update();
                aim.update();

                out.addData("Distance To Aim", "%.1f in", Alliance.get().distanceToAim(drive.getPose()));
                out.update();
            }
        } finally {
            aim.disable();
            drive.stop();
        }
    }

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }
}
