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

@Autonomous(name = "Full2TipV666", group = "main")
@Configurable
public class Full2TipV666 extends AutoCommandBase {

    /** 起点朝向（START 共用坐标，这条路线朝 180 出发）。 *//** SHOOT_TOP_2 在这条路线里的朝向。 */
    public static double SHOOT_HEADING = 70;

    /** 开局 shooter 加速多久再开门（毫秒）。 */
    public static double SPIN_UP_MS = 1000;
    /** 开局预装球开门时间（毫秒）。 */
    public static double PRELOAD_GATE_MS = 1000;
    /** 装填区停留时间（毫秒）。 */
    public static double LOAD_BOT_MS = 500;
    /** 上方一排吸球停留时间（毫秒）。 */
    public static double FLOWER_TOP_MS = 1000;
    /** flower 伸出后等多久再往前开（毫秒）。 */
    public static double FLOWER_DOWN_MS = 400;
    /** 中间一排吸球停留时间（毫秒）。 */
    public static double FLOWER_MID_MS = 1000;
    /** 到射球点后稳定多久再开门（毫秒）。 */
    public static double SETTLE_MS = 300;
    /** 每次到 SHOOT_TOP_2 的开门时间（毫秒）。 */
    public static double SHOOT_GATE_MS = 1000;

    @Override
    public Pose getStartPose() {
        return START_BOT_STRAFE.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                // 开局：自动瞄准 + shooter 加速 + transfer 一直转，原地射预装球
                startAuto(),
                waitMs(SPIN_UP_MS),
                shoot(PRELOAD_GATE_MS),

                // 下方装填区
                follow(startToPrepbot2()),
                follow(prepbot2ToLoadbot2()),
                waitMs(LOAD_BOT_MS),
                follow(loadbot2ToShoottop2()),
                waitMs(SETTLE_MS),
                shoot(SHOOT_GATE_MS),

                // 上方一排
//                flowerDown(),
                follow(shoottop2ToFlowertop()),
                waitMs(FLOWER_TOP_MS),
//                flowerUp(),
                follow(flowertopToShoottop2()),
                waitMs(SETTLE_MS),
                shoot(SHOOT_GATE_MS),

                // 中间一排
                follow(shoottop2ToPrepmid()),
//                flowerDown(),
                waitMs(FLOWER_DOWN_MS),
                follow(prepmidToFlowermid()),
                waitMs(FLOWER_MID_MS),
//                flowerUp(),
                follow(flowermidToShoottop2()),
                waitMs(SETTLE_MS),
                shoot(SHOOT_GATE_MS),

                // 停车
                follow(shoottop2ToPark()),
                stopAll()
        );
    }

    // ================= 本路线专用朝向 =================


    private Pose shootTop2() {
        return SHOOT_TOP_2.pose(SHOOT_HEADING);
    }

    // ================= 路径 =================

    public Path startToPrepbot2() {
        Pose a = START_BOT_STRAFE.pose(), b = PREP_BOT_2.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path prepbot2ToLoadbot2() {
        Pose a = PREP_BOT_2.pose(), b = LOAD_BOT_2.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path loadbot2ToShoottop2() {
        Pose a = LOAD_BOT_2.pose(), b = shootTop2();
        return Paths.curve(a, CTRL_LOAD_BOT_2_TO_SHOOT_TOP_2.pose(), b).linear(a, b);
    }

    public Path shoottop2ToFlowertop() {
        Pose a = shootTop2(), b = FLOWER_TOP.pose();
        return Paths.curve(a, CTRL_SHOOT_TOP_2_TO_FLOWER_TOP.pose(), b).linear(a, b);
    }

    public Path flowertopToShoottop2() {
        Pose a = FLOWER_TOP.pose(), b = shootTop2();
        return Paths.curve(a, CTRL_FLOWER_TOP_TO_SHOOT_TOP_2.pose(), b).linear(a, b);
    }

    public Path shoottop2ToPrepmid() {
        Pose a = shootTop2(), b = PREP_MID.pose();
        return Paths.curve(a, CTRL_SHOOT_TOP_2_TO_PREP_MID.pose(), b).linear(a, b);
    }

    public Path prepmidToFlowermid() {
        Pose a = PREP_MID.pose(), b = FLOWER_MID.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path flowermidToShoottop2() {
        Pose a = FLOWER_MID.pose(), b = shootTop2();
        return Paths.curve(a, CTRL_FLOWER_MID_TO_SHOOT_TOP_2.pose(), b).linear(a, b);
    }

    public Path shoottop2ToPark() {
        Pose a = shootTop2(), b = PARK.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
