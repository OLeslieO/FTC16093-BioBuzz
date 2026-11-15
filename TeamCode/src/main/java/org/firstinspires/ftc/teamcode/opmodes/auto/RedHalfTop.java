package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoCommandBase;

/**
 * 自动阶段：按 Pedro Pathing Visualizer 导出的路线依次跑完。
 *
 * <p>路线：start →（曲线）shoot1 → pollenPrep1 → pollen1 →（曲线）shoot2 → park。
 *
 * <p>发射：开局打开自动瞄准、shooter 加速、transfer 一直转；
 * 到 SHOOT1 后开门 SHOOT1_GATE_MS；到 SHOOT2 后开门 SHOOT2_GATE_MS。
 * 子系统、循环、位姿传递都在 {@link AutoCommandBase} 里。
 *
 * <p><b>INIT 时云台要朝正前方</b>（会把当前位置当作零点）。
 */
@Autonomous(name = "RedHalfTop", group = "main")
@Configurable
public class RedHalfTop extends AutoCommandBase {

    /** 开局 shooter 加速多久再出发（毫秒）。 */
    public static double SPIN_UP_MS = 500;
    /** 到 SHOOT1 后开门时间（毫秒）。 */
    public static double SHOOT1_GATE_MS = 2500;
    /** 到 SHOOT2 后开门时间（毫秒）。 */
    public static double SHOOT2_GATE_MS = 2500;

    // 点位都在 AutoConstants 里

    @Override
    public Pose getStartPose() {
        return START_TOP.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                // 开局：自动瞄准 + shooter 加速 + transfer 一直转
                startShooting(),
                waitMs(SPIN_UP_MS),
                follow(startToShoot1()),
                shoot(SHOOT1_GATE_MS),

                follow(shoot1ToPollenPrep1()),
                follow(pollenPrep1ToPollen1()),
                follow(pollen1ToShoot2()),
                shoot(SHOOT2_GATE_MS),

                follow(park())
        );
    }

    // ================= 路径（点位见 AutoConstants） =================

    public Path startToShoot1() {
        Pose a = START_TOP.pose(), b = SHOOT_TOP.pose(-90);
        return Paths.curve(a, CTRL_START_TOP_TO_SHOOT_TOP.pose(), b).linear(a, b);
    }

    public Path shoot1ToPollenPrep1() {
        Pose a = SHOOT_TOP.pose(-90), b = POLLEN_TOP_PREP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path pollenPrep1ToPollen1() {
        Pose a = POLLEN_TOP_PREP.pose(), b = POLLEN_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path pollen1ToShoot2() {
        Pose a = POLLEN_TOP.pose(), b = SHOOT_TOP.pose(180);
        return Paths.curve(a, CTRL_TOP_TO_SHOOT_TOP.pose(), b).linear(a, b);
    }

    public Path park() {
        Pose a = SHOOT_TOP.pose(180), b = PARK_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
