package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name = "AutoPath", group = "main")
@Configurable
public class AutoPath extends AutoCommandBase {

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
    private static final Pose START = POSE.of(56, 8, 180);
    private static final Pose SHOOT1_INTAKE1 = POSE.of(9, 8, 180);
    private static final Pose SHOOT2 = POSE.of(57, 120, 0);
    private static final Pose SHOOT2_CONTROL1 = POSE.of(23.3545, 18.6502, 0);
    private static final Pose SHOOT2_CONTROL2 = POSE.of(18.2704, 110.3779, 0);
    private static final Pose INTAKE2 = POSE.of(46.5, 120, 90);
    private static final Pose INTAKE3_1 = POSE.of(16, 46.5, 180);
    private static final Pose INTAKE3_1_CONTROL1 = POSE.of(27.8147, 114.2551, 0);

    private static final Pose INTAKE3_2 = POSE.of(12, 46.5, 180);
    private static final Pose SHOOT3 = POSE.of(56, 23, 180);

    @Override
    public Pose getStartPose() {
        return START;
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                // 开局：自动瞄准 + shooter 加速 + transfer 一直转
                startShooting(),
                waitMs(SPIN_UP_MS),
                shoot(PRELOAD_GATE_MS),

                driveTo(START, SHOOT1_INTAKE1),
                follow(Paths.curve(SHOOT1_INTAKE1, SHOOT2_CONTROL1, SHOOT2_CONTROL2, SHOOT2)
                        .linear(SHOOT1_INTAKE1, SHOOT2)),
                shoot(SHOOT2_GATE_MS),

                driveTo(SHOOT2, INTAKE2),
                follow(Paths.curve(INTAKE2, INTAKE3_1_CONTROL1, INTAKE3_1)
                        .linear(INTAKE2, INTAKE3_1)),
                driveTo(INTAKE3_1, INTAKE3_2),
                driveTo(INTAKE3_2, SHOOT3),
                shoot(SHOOT3_GATE_MS)
        );
    }
}
