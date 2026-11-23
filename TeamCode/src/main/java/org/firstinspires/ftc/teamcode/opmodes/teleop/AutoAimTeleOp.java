package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * 比赛 TeleOp：底盘（Pedro）+ 自瞄云台 + shooter + transfer + gate，全部手柄 1。
 *
 * <p>INIT：X 蓝 / B 红 选联盟。紧跟 auto 时沿用 auto 结束的位姿和云台编码器；
 * 否则位姿 = START_X/Y/HEADING，云台要朝正前方（编码器清零）。
 *
 * <ul>
 *   <li>左摇杆平移，右摇杆 x 转向；左扳机按住慢速</li>
 *   <li>右肩键：intake；左肩键：outtake（优先）</li>
 *   <li>右扳机：按住开 gate 射球，transfer 同时往里送</li>
 *   <li>自瞄一直开着；shooter 固定 SHOOTER_VEL 一直转</li>
 *   <li>dpad 左：切换瞄准点；Y：边走边射开关；A：robot / field centric 切换</li>
 *   <li>back：位姿重置为 START 位姿</li>
 * </ul>
 * 云台锁定 + shooter 到速时手柄震一下。
 */
@TeleOp(name = "AutoAim TeleOp", group = "main")
@Configurable
public class AutoAimTeleOp extends CommandOpModeEx {

    /** 没有 auto 传过来的位姿时用这个起点（红方坐标，蓝方自动镜像）。 */
    public static double START_X = 56.0;
    public static double START_Y = 8.0;
    public static double START_HEADING_DEG = 90.0;

    public static double SLOW_SCALE = 0.35;
    public static double TRIGGER_THRESHOLD = 0.5;
    public static double BRAKE_STICK_THRESHOLD = 0.05;

    /** shooter 固定转速（ticks/s），Panels 里改实时生效。 */
    public static double SHOOTER_VEL = 1700.0;
    /** true = 只有锁定且到速才开 gate（按住扳机等它就绪）。 */
    public static boolean REQUIRE_READY_TO_SHOOT = false;

    private Telemetry out;
    private Follower follower;
    private PPDriveSubsystem drive;
    private AutoAimSubsystem aim;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private GateSubsystem gate;

    private boolean poseRestored = false;

    private boolean lastDpadLeft, lastY, lastA, lastBack;
    private boolean lastReady = false;
    private double lastTime;

    @Override
    public void initialize() {
        out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (!Alliance.isSet()) {
            Alliance.set(Alliance.DEFAULT);
        }

        follower = Constants.create(hardwareMap);
        drive = new PPDriveSubsystem(follower, out);
        drive.setRobotCentric(true);
        poseRestored = drive.restorePose();
        if (!poseRestored) {
            drive.setPose(startPose());
        }

        shooter = new ShooterSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        shooter.stop();
        transfer.stop();
        gate.stop();
        register(shooter, transfer, gate);

        // 云台编码器和 transfer 的 preShooter 共用一个口，所以要在 transfer 之后建。
        // 紧跟 auto 时不清零，沿用 auto 里的角度。
        aim = new AutoAimSubsystem(hardwareMap, out, drive::getPose, drive::getVelocity, !poseRestored);
    }

    @Override
    public void functionalButtons() {
    }

    @Override
    public void initLoop() {
        Alliance previous = Alliance.get();
        Alliance.selectWithGamepad(gamepad1);
        // 换了联盟且不是 auto 传来的位姿 → 起点跟着镜像
        if (!poseRestored && Alliance.get() != previous) {
            drive.setPose(startPose());
        }
        follower.update();

        Pose p = drive.getPose();
        telemetry.addData("Alliance", Alliance.get() + "（X 蓝 / B 红）");
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)%s", p.x(), p.y(), Math.toDegrees(p.heading()),
                poseRestored ? "  来自 auto" : "  默认起点");
        telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(aim.getTurretAngle()));
        if (!poseRestored) {
            telemetry.addLine("云台要朝正前方");
        }
        telemetry.update();
    }

    @Override
    public void onStart() {
        drive.startTeleOp();
        FieldConstants.AIM_POINT_INDEX = FieldConstants.aimPointIndexForY(drive.getPose().y());
        aim.enable();
        shooter.setVelocity(SHOOTER_VEL);
        lastTime = getRuntime();
    }

    @Override
    public void run() {
        double now = getRuntime();
        double loopMs = (now - lastTime) * 1000.0;
        lastTime = now;

        // ---- 按键（上升沿）----
        if (gamepad1.dpad_left && !lastDpadLeft) {
            FieldConstants.toggleAimPoint();
        }
        if (gamepad1.y && !lastY) {
            aim.setShootOnTheMove(!aim.isShootOnTheMove());
        }
        if (gamepad1.a && !lastA) {
            drive.toggleDriveMode();
        }
        if (gamepad1.back && !lastBack) {
            drive.setPose(startPose());
        }
        lastDpadLeft = gamepad1.dpad_left;
        lastY = gamepad1.y;
        lastA = gamepad1.a;
        lastBack = gamepad1.back;

        // ---- 底盘 ----
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x; // PPDrive：+turn = 逆时针
        drive.setPowerScale(gamepad1.left_trigger > TRIGGER_THRESHOLD ? SLOW_SCALE : 1.0);
        drive.drive(forward, strafe, turn);
        aim.setBraking(Math.abs(forward) < BRAKE_STICK_THRESHOLD
                && Math.abs(strafe) < BRAKE_STICK_THRESHOLD
                && Math.abs(turn) < BRAKE_STICK_THRESHOLD);

        // ---- 射球 / transfer / gate ----
        boolean ready = aim.isLocked() && shooter.isReady();
        boolean shootHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean gateOpen = shootHeld && (!REQUIRE_READY_TO_SHOOT || ready);

        if (gamepad1.left_bumper) {
            transfer.setState(TransferSubsystem.State.OUTAKE);
        } else if (gamepad1.right_bumper || shootHeld) {
            transfer.setState(TransferSubsystem.State.INTAKE);
        } else {
            transfer.stop();
        }

        if (gateOpen && !gamepad1.left_bumper) {
            gate.shoot();
        } else {
            gate.stop();
        }


        lastReady = ready;

        // ---- 更新：先定位，再按新位姿切瞄准点、瞄准，最后 subsystem 写硬件----
        shooter.setVelocity(SHOOTER_VEL);
        drive.update();
        FieldConstants.updateAimPointForY(drive.getPose().y());
        aim.update();
        super.run();

        out.addData("Alliance", Alliance.get());
        out.addData("Aim Point", "#%d (%.1f, %.1f)", FieldConstants.AIM_POINT_INDEX,
                Alliance.get().getAimX(), Alliance.get().getAimY());
        out.addData("Ready", ready ? "READY" : "-");
        out.addData("Shooter", "%s  %.0f / %.0f", shooter.getState(),
                shooter.getVelocity(), shooter.getTargetVelocity());
        out.addData("Shot Distance", "%.1f in", aim.getShotDistance());
        out.addData("Transfer", transfer.getState());
        out.addData("Gate", gate.getState());
        out.addData("loop_ms", "%.1f", loopMs);
        out.update();
    }

    @Override
    public void end() {
        if (aim != null) {
            aim.disable();
        }
        if (drive != null) {
            drive.stop();
        }
    }

    private static Pose startPose() {
        Pose red = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        return Alliance.get().fromRed(red);
    }
}
