package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.TransferCommand;
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
 * DriveTeleOp 的复制版 + 云台自动瞄准。
 *
 * <p>自动瞄准需要场地位姿，所以底盘换成 Pedro 的 {@link PPDriveSubsystem}（robot-centric），
 * 不再用 DriveSubsystem（两个都开会抢同一组电机）。
 *
 * <p><b>INIT：</b>如果自动阶段存了位姿就沿用（云台编码器也不清零）；
 * 否则车摆在 (START_X, START_Y, START_HEADING_DEG)，云台手动摆到正前方当零点。
 * init 阶段 X 选蓝方，B 选红方。
 *
 * <p>手柄 1：
 * <ul>
 *   <li>左摇杆平移，右摇杆左右转向；按住 dpad 左键慢速</li>
 *   <li>Intake：A 伸出，B 中间，X 收回</li>
 *   <li>Transfer：按住右肩键 intake，按住左肩键 outake</li>
 *   <li>Nectar gate：右扳机按下打开，同时 intake 以 0.85 功率送球；松开关门、intake 停</li>
 *   <li>Pollen gate：左扳机按下打开，同时 intake 送球；松开关门、intake 停</li>
 *   <li>Nectar / Pollen shooter：默认一直 ACCELERATE，射速都按距离查表</li>
 *   <li>自动瞄准：开局自动打开；dpad 上 开 / 关，Y 开 / 关边走边射</li>
 *   <li>dpad 下：切换瞄准用的云台几何参数（Nectar / Pollen 旋转轴偏移，开局默认 Nectar）</li>
 *   <li>dpad 右：切换瞄准点（红方 (59, 59) / (59, 82.5)，开局默认前者）</li>
 * </ul>
 */
@TeleOp(name = "AutoAim TeleOp", group = "main")
@Configurable
public class AutoAimTeleOp extends CommandOpModeEx {

    /** 慢速模式下的功率系数。 */
    public static double SLOW_SCALE = 0.35;

    /** 右扳机超过这个值就打开 nectar gate。 */
    public static double GATE_TRIGGER_THRESHOLD = 0.5;

    /** 三个摇杆都小于这个值时认为底盘在刹车，启用刹车预测。 */
    public static double BRAKE_STICK_THRESHOLD = 0.05;

    /** 没有自动阶段传过来的位姿时用的起点。 */
    public static double START_X = 55.64;
    public static double START_Y = 7.677;
    public static double START_HEADING_DEG = 90.0;

    /** 开局是否自动打开自动瞄准。 */
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
    private GamepadEx driver;
    private Telemetry out;

    @Override
    public void initialize() {
        out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        FieldConstants.AIM_POINT_INDEX = 0; // 开局默认瞄 0 号点
        AutoAimConfig.ACTIVE_TURRET = AutoAimConfig.Turret.NECTAR; // 开局默认用 nectar 云台参数

        if (!Alliance.isSet()) {
            Alliance.set(Alliance.DEFAULT);
        }

        follower = Constants.create(hardwareMap);
        drive = new PPDriveSubsystem(follower, out);
        drive.setRobotCentric(true);
        boolean fromAuto = drive.restorePose();
        if (!fromAuto) {
            drive.setPose(startPose());
        }

        // 从自动阶段接过来时云台编码器沿用自动阶段的读数，不清零
        aim = new AutoAimSubsystem(hardwareMap, out, drive::getPose, drive::getVelocity, !fromAuto);

        transfer = new TransferSubsystem(hardwareMap);
        nectarGate = new NectarGateSubsystem(hardwareMap);
        pollenGate = new PollenGateSubsystem(hardwareMap);
        nectarShooter = new NectarShooterSubsystem(hardwareMap);
        pollenShooter = new PollenShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        nectarShooter.accelerate();
        // 射速按距离查表：自瞄开着用解算出的射击距离（边走边射时已含提前量），否则用到瞄准点的直线距离
        nectarShooter.setDistanceSupplier(() -> aim.isEnabled() && aim.getShotDistance() > 0
                ? aim.getShotDistance()
                : Alliance.get().distanceToAim(drive.getPose()));
        pollenShooter.accelerate();
        pollenShooter.setDistanceSupplier(() -> aim.isEnabled() && aim.getShotDistance() > 0
                ? aim.getShotDistance()
                : Alliance.get().distanceToAim(drive.getPose()));
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
        out.addData("Pose From Auto", PPDriveSubsystem.hasTransferredPose());
        out.addData("Turret Angle", "%.1f°", Math.toDegrees(aim.getTurretAngle()));
        out.update();
    }

    @Override
    public void functionalButtons() {

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new TransferCommand(transfer, TransferSubsystem.State.INTAKE));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new TransferCommand(transfer, TransferSubsystem.State.OUTAKE));

        // 右扳机按下开门 + intake 以 TransferSubsystem.SHOOT_POW 送球，松开关门、intake 停
        new Trigger(() -> gamepad1.right_trigger > GATE_TRIGGER_THRESHOLD)
                .whileActiveOnce(new ParallelCommandGroup(
                        new StartEndCommand(nectarGate::shoot, nectarGate::stop, nectarGate),
                        new TransferCommand(transfer, TransferSubsystem.State.SHOOT)));
        new Trigger(() -> gamepad1.left_trigger > GATE_TRIGGER_THRESHOLD)
                .whileActiveOnce(new ParallelCommandGroup(
                        new StartEndCommand(pollenGate::shoot, pollenGate::stop, pollenGate),
                        new TransferCommand(transfer, TransferSubsystem.State.SHOOT)));


        // 自动瞄准开关 / 边走边射开关 / 切换云台参数 / 切换瞄准点
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (aim.isEnabled()) {
                        aim.disable();
                    } else {
                        aim.enable();
                    }
                }));
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> aim.setShootOnTheMove(!aim.isShootOnTheMove())));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(AutoAimConfig::toggleTurret));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(FieldConstants::toggleAimPoint));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeCommand(intake, IntakeSubsystem.State.EXTENDED));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new IntakeCommand(intake, IntakeSubsystem.State.MIDDLE));
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new IntakeCommand(intake, IntakeSubsystem.State.RETRACTED));
    }

    @Override
    public void onStart() {
        drive.startTeleOp();
        if (AIM_ON_START) {
            aim.enable();
        }
    }

    @Override
    public void run() {
        // 按键命令 + intake / transfer / gate / shooter 的 periodic
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

        // 先更新定位，再用新位姿瞄准
        drive.update();
        aim.update();

        out.addData("nectar target", nectarShooter.getTargetVelocity());
        out.addData("shooter velocity", nectarShooter.getVelocity());
        out.addData("shooter error", nectarShooter.getError());
        out.addData("pollen target", pollenShooter.getTargetVelocity());
        out.addData("pollen table distance", "%.1f in", pollenShooter.getDistance());
        out.addData("pollen velocity", pollenShooter.getVelocity());
        out.addData("pollen error", pollenShooter.getError());
        out.addData("aim turret", AutoAimConfig.ACTIVE_TURRET);
        out.addData("aim enabled", aim.isEnabled());
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

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }
}
