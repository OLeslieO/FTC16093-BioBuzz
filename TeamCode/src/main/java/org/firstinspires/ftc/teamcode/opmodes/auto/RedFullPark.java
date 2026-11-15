package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
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
@Autonomous(name = "RedFullPark", group = "main")
@Configurable
public class RedFullPark extends AutoCommandBase {

    /** 开局 shooter 加速多久再开门（毫秒）。 */
    public static double SPIN_UP_MS = 1000;
    /** 开局预装球开门时间（毫秒）。 */
    public static double PRELOAD_GATE_MS = 2500;
    /** 到 SHOOT2 后开门时间（毫秒）。 */
    public static double SHOOT2_GATE_MS = 2500;
    /** 到 SHOOT3 后开门时间（毫秒）。 */
    public static double SHOOT3_GATE_MS = 2500;

    // 点位都在 AutoConstants 里

    @Override
    public Pose getStartPose() {
        return START_BOTTOM.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                // 开局：自动瞄准 + shooter 加速 + transfer 一直转
                startShooting(),
                waitMs(SPIN_UP_MS),
                follow(startToShoot1()),
                shoot(PRELOAD_GATE_MS),

                follow(shoot1ToPollenPrep1()),
                flowerDown(),
                waitMs(500),
                follow(pollenPrep1ToPollen1()),
                waitMs(1000),
                follow(shoot2PrepToShoot2()),
                follow(shoot2PrepToShoot2()),
                flowerUp(),
                waitMs(200),
                shoot(SHOOT2_GATE_MS),

                follow(shoot2ToPollenPrep2()),
                flowerDown(),
                waitMs(500),
                follow(pollenPrep2ToPollen2()),
                waitMs(1000),
                follow(pollen2ToShoot3()),
                flowerUp(),
                waitMs(500),
                shoot(SHOOT3_GATE_MS),

                follow(park())
        );
    }

    // ================= 路径（点位见 AutoConstants） =================

    public Path startToShoot1() {
        Pose a = START_BOTTOM.pose(), b = SHOOT_BOTTOM.pose(90);
        return Paths.line(a, b).linear(a, b);
    }

    public Path shoot1ToPollenPrep1() {
        Pose a = SHOOT_BOTTOM.pose(90), b = POLLEN_MID_PREP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path pollenPrep1ToPollen1() {
        Pose a = POLLEN_MID_PREP.pose(), b = POLLEN_MID.pose();
        return Paths.line(a, b).linear(a, b);
    }
    public Path pollen1ToShoot2Prep() {
        Pose a = POLLEN_MID.pose(), b = POLLEN_MID_PREP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path shoot2PrepToShoot2() {
        return Paths.curve(POLLEN_MID_PREP.pose(), CTRL_MID_TO_SHOOT_TOP.pose(), SHOOT_TOP.pose(270))
                .heading(Interpolator.piecewise()
                        .until(0.2, Interpolator.constant(SHOOT_TOP.pose(180)))
                        .until(1, Interpolator.linear(SHOOT_TOP.pose(180), SHOOT_TOP.pose(-90))));
    }

    public Path shoot2ToPollenPrep2() {
        Pose a = SHOOT_TOP.pose(-90), b = POLLEN_TOP_PREP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path pollenPrep2ToPollen2() {
        Pose a = POLLEN_TOP_PREP.pose(), b = POLLEN_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path pollen2ToShoot3() {
        Pose a = POLLEN_TOP.pose(), b = SHOOT_TOP.pose(180);
        return Paths.curve(a, CTRL_TOP_TO_SHOOT_TOP.pose(), b).linear(a, b);
    }

    public Path park() {
        Pose a = SHOOT_TOP.pose(180), b = PARK_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
