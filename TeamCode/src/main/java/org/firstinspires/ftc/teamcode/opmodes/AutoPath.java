package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.PPDriveSubsystem;

/**
 * 自动阶段：按 Pedro Pathing Visualizer 导出的路线依次跑完。
 *
 * <p>路线：start → shoot1/intake1 → shoot2（曲线）→ intake2 → intake3（两段）→ shoot3。
 * 路径由 Ivy 调度器执行；结束时把位姿存给 TeleOp。
 */
@Autonomous(name = "AutoPath", group = "main")
public class AutoPath extends CommandOpModeEx {

    private static final PoseFactory POSE = PoseFactory.degrees();

    // ---- 关键点（英寸 / 度，Visualizer 坐标）----
    private static final Pose START = POSE.of(56, 8, 180);
    private static final Pose SHOOT1_INTAKE1 = POSE.of(9, 8, 180);
    private static final Pose SHOOT2 = POSE.of(40, 124, 90);
    private static final Pose SHOOT2_CONTROL1 = POSE.of(23.3545, 18.6502, 0);
    private static final Pose SHOOT2_CONTROL2 = POSE.of(25.6256, 97.4649, 0);
    private static final Pose INTAKE2 = POSE.of(46.5, 120, 90);
    private static final Pose INTAKE3_1 = POSE.of(14, 46.5, 180);
    private static final Pose INTAKE3_2 = POSE.of(12, 46.5, 180);
    private static final Pose SHOOT3 = POSE.of(45.632, 16.2063, 180);

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
        follower.update();
    }

    @Override
    public void functionalButtons() {
    }

    @Override
    public void onStart() {
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        Scheduler.execute();

        Pose pose = follower.pose();
        panelsTelemetry.addData("x", pose.x());
        panelsTelemetry.addData("y", pose.y());
        panelsTelemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        if (follower.currentPath() != null) {
            panelsTelemetry.addData("path index", follower.pathIndex());
            panelsTelemetry.addData("distance remaining", follower.distanceToEndpoint());
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        // 把结束位姿留给 TeleOp
        if (follower != null) {
            PPDriveSubsystem.saveTransferredPose(follower.pose());
        }
        Scheduler.reset();
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, line(START, SHOOT1_INTAKE1)),
                follow(follower, Paths.curve(SHOOT1_INTAKE1, SHOOT2_CONTROL1, SHOOT2_CONTROL2, SHOOT2)
                        .linear(SHOOT1_INTAKE1, SHOOT2)),
                follow(follower, line(SHOOT2, INTAKE2)),
                follow(follower, line(INTAKE2, INTAKE3_1)),
                follow(follower, line(INTAKE3_1, INTAKE3_2)),
                follow(follower, line(INTAKE3_2, SHOOT3))
        );
    }

    /** 直线 + 线性插值朝向。 */
    private static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }
}
