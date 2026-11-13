package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoCommandBase;

/**
 * 自动阶段：按 Pedro Pathing Visualizer 导出的路线依次跑完。
 *
 * <p>路线：start → shoot1/intake1 → shoot2（曲线）→ intake2 → intake3（两段）→ shoot3。
 * 路径由 Ivy 调度器执行；结束时把位姿存给 TeleOp。
 *
 * <p>发射：开局打开自动瞄准、shooter 加速、transfer 一直转（整个自动阶段都不停）；
 * 开局等 SPIN_UP_MS 后开门 PRELOAD_GATE_MS；到 SHOOT2 后开门 SHOOT2_GATE_MS；
 * 到 SHOOT3 后开门 SHOOT3_GATE_MS。子系统、循环、位姿传递都在 {@link AutoCommandBase} 里。
 *
 * <p><b>INIT 时云台要朝正前方</b>（会把当前位置当作零点）。
 */
@Autonomous(name = "RedFull", group = "main")
@Configurable
public class RedFull extends AutoCommandBase {

    /** 开局 shooter 加速多久再开门（毫秒）。 */
    public static double SPIN_UP_MS = 500;
    /** 开局预装球开门时间（毫秒）。 */
    public static double PRELOAD_GATE_MS = 2500;
    /** 到 SHOOT2 后开门时间（毫秒）。 */
    public static double SHOOT2_GATE_MS = 2500;
    /** 到 SHOOT3 后开门时间（毫秒）。 */
    public static double SHOOT3_GATE_MS = 2500;

    private static final PoseFactory POSE = PoseFactory.degrees();

    // ---- 关键点（英寸 / 度，Visualizer 坐标）----
    private final Pose start = POSE.of(55.64, 7.677, 90);
    private final Pose startToShoot1 = POSE.of(55.84, 23.6, 90);
    private final Pose shoot1ToPollenPrep1Start = POSE.of(55.84, 23.6, 90);
    private final Pose shoot1ToPollenPrep1 = POSE.of(19.8256, 46.75, 180);
    private final Pose pollenPrep1ToPollen1Start = POSE.of(19.8256, 46.75, 180);
    private final Pose pollenPrep1ToPollen1 = POSE.of(12.8, 46.75, 180);
    private final Pose pollen1ToShoot2 = POSE.of(55.84, 118, 270);
    private final Pose pollen1ToShoot2Control1 = POSE.of(23.2677, 118.5011, 0);
    private final Pose pollen1ToShoot2Segment1Heading = POSE.of(55.84, 118, 180);
    private final Pose pollen1ToShoot2Segment2Start = POSE.of(55.84, 118, 180);
    private final Pose pollen1ToShoot2Segment2End = POSE.of(55.84, 118, -90);
    private final Pose shoot2ToPollenPrep2Start = POSE.of(55.84, 118, -90);
    private final Pose shoot2ToPollenPrep2 = POSE.of(46.8836, 121.3282, 90);
    private final Pose pollenPrep2ToPollen2 = POSE.of(46.88, 128, 90);
    private final Pose pollen2ToShoot3Start = POSE.of(46.88, 128, 90);
    private final Pose pollen2ToShoot3 = POSE.of(55.84, 118, 180);
    private final Pose pollen2ToShoot3Control1 = POSE.of(46.9869, 118.4687, 0);
    private final Pose shoot3ToPollen3Start = POSE.of(55.84, 118, 180);
    private final Pose shoot3ToPollen3 = POSE.of(9.0816, 14.2962, -90);
    private final Pose shoot3ToPollen3Control1 = POSE.of(16.8889, 118.9597, 0);
    private final Pose shoot3ToPollen3Control2 = POSE.of(22.708, 22.5241, 0);
    private final Pose pollen3ToShoot4 = POSE.of(55.84, 23.6, -180);
    private final Pose pollen3ToShoot4Segment1Start = POSE.of(55.84, 23.6, -90);
    private final Pose pollen3ToShoot4Segment1End = POSE.of(55.84, 23.6, -90);
    private final Pose pollen3ToShoot4Segment2Start = POSE.of(55.84, 23.6, -90);
    private final Pose pollen3ToShoot4Segment2End = POSE.of(55.84, 23.6, 180);

    @Override
    public Pose getStartPose() {
        return start;
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                // 开局：自动瞄准 + shooter 加速 + transfer 一直转
                startShooting(),
                waitMs(SPIN_UP_MS),
                driveTo(start,startToShoot1),
                shoot(PRELOAD_GATE_MS),

                driveTo(shoot1ToPollenPrep1Start, shoot1ToPollenPrep1),
                driveTo(pollenPrep1ToPollen1Start, pollenPrep1ToPollen1),
                follow(Paths.curve(pollenPrep1ToPollen1, pollen1ToShoot2Control1, pollen1ToShoot2).heading(Interpolator.piecewise().until(0.2, Interpolator.constant(pollen1ToShoot2Segment1Heading)).until(1, Interpolator.linear(pollen1ToShoot2Segment2Start, pollen1ToShoot2Segment2End)))),
                shoot(SHOOT2_GATE_MS),

                driveTo(shoot2ToPollenPrep2Start, shoot2ToPollenPrep2),
                driveTo(shoot2ToPollenPrep2, pollenPrep2ToPollen2),
                follow(Paths.curve(pollen2ToShoot3Start, pollen2ToShoot3Control1, pollen2ToShoot3).linear(pollen2ToShoot3Start, pollen2ToShoot3)),
                shoot(SHOOT3_GATE_MS),
                follow(Paths.curve(shoot3ToPollen3Start, shoot3ToPollen3Control1, shoot3ToPollen3Control2, shoot3ToPollen3).linear(shoot3ToPollen3Start, shoot3ToPollen3)),
                driveTo(shoot3ToPollen3, pollen3ToShoot4)
        );
    }
}
