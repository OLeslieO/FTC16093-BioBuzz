package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.NectarGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PollenGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimConfig;
import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * 双手柄版 {@link TeleOpSingle}：手柄 1 的操作完全一样，另外手柄 2 的左肩键也能 outtake。
 *
 * <p>自动瞄准需要场地位姿，所以底盘换成 Pedro 的 {@link PPDriveSubsystem}（robot-centric），
 * 不再用 DriveSubsystem（两个都开会抢同一组电机）。
 *
 * <p><b>INIT：</b>如果自动阶段存了位姿就沿用（云台编码器也不清零），读完就清掉，下次再开 TeleOp 不会再用；
 * 否则车摆在 (START_X, START_Y, START_HEADING_DEG)，云台手动摆到正前方当零点。
 * init 阶段 X 选蓝方，B 选红方。
 *
 * <p>手柄 1：
 * <ul>
 *   <li>左摇杆平移，右摇杆左右转向；按住 dpad 左键慢速</li>
 *   <li>Y：flower 在 bottom / middle 之间切换（intake 放下时才生效）</li>
 *   <li>A：intake 在 middle / retracted 之间切换；开局默认 middle
 *       （flower 和 intake 的先后顺序按 {@link IntakeSubsystem} 的联动规则）</li>
 *   <li>左肩键：在 Pollen / Nectar 之间切换（开局默认 Pollen），瞄准云台和加速的 shooter 一起切；
 *       切换后对应 shooter 第一次转到它的 MIN_SHOT_VEL 时，手柄震动 SPEED_RUMBLE_MS</li>
 *   <li>右扳机：开当前云台对应的 gate 并送球（Pollen 功率 POLLEN_FEED_POW，Nectar 功率 NECTAR_FEED_POW）</li>
 *   <li>左扳机：outtake（功率 TransferSubsystem.OUTAKE_POW）</li>
 *   <li>右肩键：transfer 以 INTAKE_FEED_POW 吸球</li>
 *   <li>同时按时优先级：右扳机 &gt; 左扳机（或手柄 2 左肩键）&gt; 右肩键</li>
 *   <li>dpad 下：把位姿重置到 (START_X, START_Y, START_HEADING_DEG)（车要先摆到那里）</li>
 *   <li>dpad 上：云台锁到 0°（车头正前方），再按一次恢复自动瞄准</li>
 *   <li>没被选中的 shooter IDLE（INACTIVE_SHOOTER_IDLE = false 时改为 STOP）</li>
 *   <li>瞄准点自动按半场选：车在 y &lt; 70.75 瞄 (59, 59)，y &gt; 70.75 瞄 (59, 82.5)（红方）</li>
 * </ul>
 *
 * <p>手柄 2：
 * <ul>
 *   <li>左肩键：outtake（和手柄 1 左扳机一样）</li>
 * </ul>
 */
@TeleOp(name = "TeleOp Dual", group = "main")
@Configurable
public class TeleOpDual extends CommandOpModeEx {

    public static double SLOW_SCALE = 0.35;

    public static double GATE_TRIGGER_THRESHOLD = 0.5;
    public static double OUTTAKE_TRIGGER_THRESHOLD = 0.5;

    /** 射 Pollen 时 transfer 送球功率。 */
    public static double POLLEN_FEED_POW = 1.0;
    /** 射 Nectar 时 transfer 送球功率。 */
    public static double NECTAR_FEED_POW = 0.9;
    /** 吸球时 transfer 功率。 */
    public static double INTAKE_FEED_POW = 0.85;

    /** 切换 shooter 后第一次到达最低射速时手柄震动的时长（毫秒）。 */
    public static int SPEED_RUMBLE_MS = 500;

    /** dpad 上锁定云台的角度（相对车头，度）。 */
    public static double TURRET_HOLD_DEG = 0.0;

    /** 三个摇杆都小于这个值时认为底盘在刹车，启用刹车预测。 */
    public static double BRAKE_STICK_THRESHOLD = 0.05;

    /** 没有自动阶段传过来的位姿时用的起点。 */
    public static double START_X = 55.64;
    public static double START_Y = 7.677;
    public static double START_HEADING_DEG = 90.0;

    /** 没被选中的那个 shooter：true = IDLE 低速空转（切换后到速更快），false = STOP。 */
    public static boolean INACTIVE_SHOOTER_IDLE = true;

    public static boolean AIM_ON_START = true;

    private Follower follower;
    private PPDriveSubsystem drive;
    private AutoAimSubsystem aim;
    private TransferSubsystem transfer;
    private NectarGateSubsystem nectarGate;
    private PollenGateSubsystem pollenGate;
    private NectarShooterSubsystem nectarShooter;
    private PollenShooterSubsystem pollenShooter;

    private IntakeSubsystem intake;

    /** 上一个循环瞄准的云台，用来发现切换。 */
    private AutoAimConfig.Turret lastTurret;
    /** 切换后还没到最低射速，到了就震动一次。 */
    private boolean waitingForSpeed = false;
    private GamepadEx driver;
    private Telemetry out;
    /** INIT 时是否用了自动阶段传过来的位姿（读完开关就清掉了，所以单独记一下）。 */
    private boolean fromAuto;

    @Override
    public void initialize() {
        out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        AutoAimConfig.ACTIVE_TURRET = AutoAimConfig.Turret.POLLEN; // 开局默认 pollen（左肩键切换）

        if (!Alliance.isSet()) {
            Alliance.set(Alliance.DEFAULT);
        }

        follower = Constants.create(hardwareMap);
        drive = new PPDriveSubsystem(follower, out);
        drive.setRobotCentric(true);
        fromAuto = drive.restorePose();
        if (!fromAuto) {
            drive.setPose(startPose());
        }
        // 瞄准点按车所在半场自动选
        FieldConstants.AIM_POINT_INDEX = FieldConstants.aimPointIndexForY(drive.getPose().y());

        // 从自动阶段接过来时云台编码器沿用自动阶段的读数，不清零
        aim = new AutoAimSubsystem(hardwareMap, out, drive::getPose, drive::getVelocity, !fromAuto);

        transfer = new TransferSubsystem(hardwareMap);
        nectarGate = new NectarGateSubsystem(hardwareMap);
        pollenGate = new PollenGateSubsystem(hardwareMap);
        nectarShooter = new NectarShooterSubsystem(hardwareMap);
        pollenShooter = new PollenShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        nectarShooter.setDistanceSupplier(() -> aim.isEnabled() && aim.getShotDistance() > 0
                ? aim.getShotDistance()
                : Alliance.get().distanceToAim(drive.getPose()));
        pollenShooter.setDistanceSupplier(() -> aim.isEnabled() && aim.getShotDistance() > 0
                ? aim.getShotDistance()
                : Alliance.get().distanceToAim(drive.getPose()));
        updateShooterStates();
        register(transfer, nectarGate, pollenGate, nectarShooter, pollenShooter, intake);

        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void initLoop() {
        Alliance.selectWithGamepad(gamepad1);
        follower.update();

        Pose p = drive.getPose();
        out.addData("Alliance", Alliance.get() + "（X 蓝 / B 红）");
        out.addData("Pose", "(%.1f, %.1f, %.1f°)", p.x(), p.y(), Math.toDegrees(p.heading()));
        out.addData("Pose From Auto", fromAuto);
        out.addData("Turret Angle", "%.1f°", Math.toDegrees(aim.getTurretAngle()));
        out.update();
    }

    @Override
    public void functionalButtons() {
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(intake::toggleFlower, intake));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    if (intake.getTargetState() == IntakeSubsystem.State.RETRACTED) {
                        intake.middle();
                    } else {
                        intake.retract();
                    }
                }, intake));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> drive.setPose(startPose())));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (aim.isHolding()) {
                        aim.releaseHold();
                    } else {
                        aim.holdAngle(Math.toRadians(TURRET_HOLD_DEG));
                    }
                }));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    AutoAimConfig.toggleTurret();
                    updateShooterStates();
                }));
    }

    @Override
    public void onStart() {
        drive.startTeleOp();
        if (AIM_ON_START) {
            aim.enable();
        }
        intake.middle();
    }

    @Override
    public void run() {
        updateShooterStates();
        updateFeed();
        updateSpeedRumble();

        super.run();

        // ---- 底盘 ----
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x; // PPDrive：+turn = 逆时针
        drive.setPowerScale(gamepad1.dpad_left ? SLOW_SCALE : 1.0);
        drive.drive(forward, strafe, turn);

        aim.setBraking(Math.abs(forward) < BRAKE_STICK_THRESHOLD
                && Math.abs(strafe) < BRAKE_STICK_THRESHOLD
                && Math.abs(turn) < BRAKE_STICK_THRESHOLD);

        // 先更新定位，再按新位姿选瞄准点、瞄准
        drive.update();
        FieldConstants.updateAimPointForY(drive.getPose().y());
        aim.update();

        Pose pose = drive.getPose();
        out.addData("pose", "(%.1f, %.1f, %.1f°)", pose.x(), pose.y(), Math.toDegrees(pose.heading()));

        out.addData("intake", "%s（目标 %s）", intake.getState(), intake.getTargetState());
        out.addData("flower", "%s → %.2f", intake.getFlowerState(), intake.getFlowerTargetPosition());

        out.addData("nectar target", nectarShooter.getTargetVelocity());
        out.addData("nectar velocity", nectarShooter.getVelocity());
        out.addData("nectar error", nectarShooter.getError());
        out.addData("nectar table distance", "%.1f in", nectarShooter.getDistance());

        out.addData("pollen target", pollenShooter.getTargetVelocity());
        out.addData("pollen table distance", "%.1f in", pollenShooter.getDistance());
        out.addData("pollen velocity", pollenShooter.getVelocity());
        out.addData("pollen error", pollenShooter.getError());
        out.addData("aim turret", AutoAimConfig.ACTIVE_TURRET);
        out.addData("aim shoot on the move", aim.isShootOnTheMove());
        out.addData("aim point", "#%d (%.1f, %.1f)", FieldConstants.AIM_POINT_INDEX,
                Alliance.get().getAimX(), Alliance.get().getAimY());
        out.addData("distance to aim", "%.1f in", Alliance.get().distanceToAim(drive.getPose()));
        out.update();
    }

    @Override
    public void end() {
        aim.disable();
        drive.stop();
        super.end();
    }

    /** 当前瞄准用的云台对应的 shooter 加速，另一个 IDLE / STOP。 */
    private void updateShooterStates() {
        boolean pollenActive = AutoAimConfig.ACTIVE_TURRET == AutoAimConfig.Turret.POLLEN;
        if (pollenActive) {
            pollenShooter.accelerate();
            if (INACTIVE_SHOOTER_IDLE) nectarShooter.idle(); else nectarShooter.stop();
        } else {
            nectarShooter.accelerate();
            if (INACTIVE_SHOOTER_IDLE) pollenShooter.idle(); else pollenShooter.stop();
        }
    }

    /**
     * 右扳机：开当前云台的 gate + 按云台选送球功率；否则左扳机 / 手柄 2 左肩键 outtake；
     * 否则右肩键吸球；都没按就停。
     * 没在用的 gate 一直关着。
     */
    private void updateFeed() {
        boolean shooting = gamepad1.right_trigger > GATE_TRIGGER_THRESHOLD;
        boolean pollen = AutoAimConfig.ACTIVE_TURRET == AutoAimConfig.Turret.POLLEN;

        if (shooting && pollen) pollenGate.shoot(); else pollenGate.stop();
        if (shooting && !pollen) nectarGate.shoot(); else nectarGate.stop();

        if (shooting) {
            transfer.setPower(pollen ? POLLEN_FEED_POW : NECTAR_FEED_POW);
        } else if (gamepad1.left_trigger > OUTTAKE_TRIGGER_THRESHOLD || gamepad2.left_bumper) {
            transfer.setState(TransferSubsystem.State.OUTAKE);
        } else if (gamepad1.right_bumper) {
            transfer.setPower(INTAKE_FEED_POW);
        } else {
            transfer.stop();
        }
    }

    /** 切换云台后，对应 shooter 第一次到达它的 MIN_SHOT_VEL 时震动一次手柄。 */
    private void updateSpeedRumble() {
        AutoAimConfig.Turret turret = AutoAimConfig.ACTIVE_TURRET;
        if (lastTurret != null && turret != lastTurret) {
            waitingForSpeed = true;
        }
        lastTurret = turret;

        if (!waitingForSpeed) {
            return;
        }
        boolean pollen = turret == AutoAimConfig.Turret.POLLEN;
        double velocity = Math.abs(pollen ? pollenShooter.getVelocity() : nectarShooter.getVelocity());
        double minVelocity = pollen
                ? PollenShooterSubsystem.MIN_SHOT_VEL
                : NectarShooterSubsystem.MIN_SHOT_VEL;
        if (velocity >= minVelocity) {
            gamepad1.rumble(SPEED_RUMBLE_MS);
            waitingForSpeed = false;
        }
    }

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }
}
