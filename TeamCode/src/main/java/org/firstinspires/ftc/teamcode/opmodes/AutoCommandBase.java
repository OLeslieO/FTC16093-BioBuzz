package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PollenGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * 所有自动程序的基类（参考 FTC2026-16093National 的 AutoCommandBase）。
 *
 * <p>子类只需要实现：
 * <ul>
 *   <li>{@link #getStartPose()}：起始位姿（可以按 {@code Alliance.get()} 返回红/蓝不同的点）</li>
 *   <li>{@link #runAutoCommand()}：整条自动流程（Ivy Command）</li>
 * </ul>
 *
 * <p>基类负责：建 follower、建好 shooter / transfer / nectarGate / intake / 自瞄、
 * INIT 阶段选联盟、START 时重新设起点并调度流程、每循环更新 follower → Ivy → 自瞄、
 * 结束时把位姿留给 TeleOp。
 *
 * <p>SolversLib 子系统的 periodic 由 {@code super.run()} 调用；路径/流程用 Ivy {@link Scheduler}。
 *
 * <p><b>INIT 时云台要朝正前方</b>（会把当前位置当作零点）。
 */
public abstract class AutoCommandBase extends CommandOpModeEx {

    protected Follower follower;
    protected TelemetryManager panelsTelemetry;
    protected Telemetry out;

    protected NectarShooterSubsystem shooter;
    protected TransferSubsystem transfer;
    protected PollenGateSubsystem gate;

    protected IntakeSubsystem intake;
    protected AutoAimSubsystem aim;

    /** 整条自动流程。在 START 时调用，此时联盟已经选好。 */
    public abstract Command runAutoCommand();

    /** 起始位姿。INIT 和 START 时各设一次（START 时联盟已确定）。 */
    public abstract Pose getStartPose();

    /** 子类额外初始化（在基类 initialize 之后调用），默认什么都不做。 */
    public void onInit() {
    }

    /** 每个循环额外执行的逻辑 / telemetry（在 panelsTelemetry.update 之前），默认什么都不做。 */
    public void periodic() {
    }

    /** 自动结束时调用（跑完或在 DS 上停止），默认什么都不做。 */
    public void onAutoStopped() {
    }

    @Override
    public final void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Scheduler.reset();

        if (!Alliance.isSet()) {
            Alliance.set(Alliance.DEFAULT);
        }

        follower = Constants.create(hardwareMap);
        follower.setPose(getStartPose());
        follower.update();

        shooter = new NectarShooterSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        gate = new PollenGateSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        // init 时先停着，按 START 再动
        shooter.stop();
        transfer.stop();
        gate.stop();
        register(shooter, transfer, gate, intake);

        // 云台在 INIT 时朝正前方，当作零点
        aim = new AutoAimSubsystem(hardwareMap, out, follower::pose, follower::velocity, true);
        // 射速按距离查表：自瞄有解时用射击距离，否则用到瞄准点的直线距离
        shooter.setDistanceSupplier(() -> aim.isEnabled() && aim.getShotDistance() > 0
                ? aim.getShotDistance()
                : Alliance.get().distanceToAim(follower.pose()));

        onInit();
    }

    @Override
    public void initLoop() {
        Alliance.selectWithGamepad(gamepad1);
        telemetry.addData("Alliance", Alliance.get() + "（X 蓝 / B 红）");
        telemetry.addLine("云台要朝正前方");
        telemetry.update();
    }

    @Override
    public void functionalButtons() {
    }

    @Override
    public final void onStart() {
        // 联盟可能在 init 阶段改了，按最终联盟重设起点
        follower.setPose(getStartPose());
        follower.update();
        Scheduler.schedule(runAutoCommand());
    }

    @Override
    public final void run() {
        // SolversLib 子系统的 periodic
        super.run();

        follower.update();
        Scheduler.execute();
        // 先更新定位，再用新位姿瞄准
        aim.update();

        periodic();

        Pose pose = follower.pose();
        panelsTelemetry.addData("x", pose.x());
        panelsTelemetry.addData("y", pose.y());
        panelsTelemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        panelsTelemetry.addData("shooter velocity", shooter.getVelocity());
        panelsTelemetry.addData("shooter ready", shooter.isReady());
        panelsTelemetry.addData("transfer state", transfer.getState());
        panelsTelemetry.addData("gate state", gate.getState());
        panelsTelemetry.addData("aim locked", aim.isLocked());
        if (follower.currentPath() != null) {
            panelsTelemetry.addData("path index", follower.pathIndex());
            panelsTelemetry.addData("distance remaining", follower.distanceToEndpoint());
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public final void end() {
        try {
            onAutoStopped();
        } finally {
            // 把结束位姿留给 TeleOp
            if (follower != null) {
                PPDriveSubsystem.saveTransferredPose(follower.pose());
            }
            if (aim != null) {
                aim.disable();
            }
            Scheduler.reset();
        }
    }

    // ================= 常用小命令 =================

    /** 跟一条路径。 */
    protected Command follow(Path path) {
        return PedroCommands.follow(follower, path);
    }

    /** 直线 + 线性插值朝向，直接跟。 */
    protected Command driveTo(Pose from, Pose to) {
        return follow(line(from, to));
    }

    /** 开自瞄 + shooter 加速 + transfer 一直转。 */
    protected Command startShooting() {
        return instant(() -> {
            aim.enable();
            shooter.accelerate();
            transfer.setState(TransferSubsystem.State.INTAKE);
            intake.setState(IntakeSubsystem.State.EXTENDED);
        });
    }

    /** 射球 ms 毫秒再关上。 */
    protected Command shoot(double ms) {
        return sequential(
                instant(gate::shoot),
                waitMs(ms),
                instant(gate::stop)
        );
    }

    /** 全部停下。 */
    protected Command stopAll() {
        return instant(() -> {
            shooter.stop();
            transfer.stop();
            gate.stop();
            aim.disable();
        });
    }

    // ================= 路径工具 =================

    /** 直线 + 线性插值朝向。 */
    protected static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }
}
