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
import org.firstinspires.ftc.teamcode.subsystem.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.autoaim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;


public abstract class AutoCommandBase extends CommandOpModeEx {

    protected Follower follower;
    protected TelemetryManager panelsTelemetry;
    protected Telemetry out;

    protected ShooterSubsystem shooter;
    protected TransferSubsystem transfer;
    protected GateSubsystem gate;

    protected FlowerSubsystem intake;
    protected AutoAimSubsystem aim;

    public abstract Command runAutoCommand();

    public abstract Pose getStartPose();
    public void onInit() {
    }

    public void periodic() {
    }

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

        shooter = new ShooterSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        intake = new FlowerSubsystem(hardwareMap);
        // init 时先停着，按 START 再动
        shooter.stop();
        transfer.stop();
        gate.stop();
        register(shooter, transfer, gate, intake);

        aim = new AutoAimSubsystem(hardwareMap, out, follower::pose, follower::velocity, true);
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
        // 按起点所在半场选红方瞄准点（下半场 0 号，上半场 1 号），之后每个循环按 y 自动切
        FieldConstants.AIM_POINT_INDEX = FieldConstants.aimPointIndexForY(follower.pose().y());
        Scheduler.schedule(runAutoCommand());
    }

    @Override
    public final void run() {
        // SolversLib 子系统的 periodic
        super.run();

        follower.update();
        Scheduler.execute();
        // 先更新定位，再按新位姿所在半场切瞄准点，最后瞄准
        FieldConstants.updateAimPointForY(follower.pose().y());
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
        panelsTelemetry.addData("aim point", FieldConstants.AIM_POINT_INDEX);
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
    protected Command startAuto() {
        return instant(() -> {
            aim.enable();
            shooter.accelerate();
            transfer.setState(TransferSubsystem.State.INTAKE);
            flowerUp();
        });
    }
    protected Command stopTransfer() {
        return instant(() -> {
            transfer.setState(TransferSubsystem.State.STOP);
        });
    }

    /** flower 伸出。 */
    protected Command flowerDown() {
        return instant(intake::extend);
    }

    /** flower 收回。 */
    protected Command flowerUp() {
        return instant(intake::retract);
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
        });
    }

    // ================= 路径工具 =================

    /** 直线 + 线性插值朝向。 */
    protected static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }
}
