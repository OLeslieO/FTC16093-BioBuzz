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
//最后吸角落预置
@Autonomous(name = "Full2Tip", group = "main")
@Configurable
public class Full2Tip extends AutoCommandBase {

    /** 开局 shooter 加速多久再开门（毫秒）。 */
    public static double SPIN_UP_MS = 1000;
    /** 开局预装球开门时间（毫秒）。 */
    public static double PRELOAD_GATE_MS = 1000;
    /** flower 伸出后等多久再往前开（毫秒）。 */
    public static double FLOWER_DOWN_MS = 400;
    /** 中间一排吸球停留时间（毫秒）。 */
    public static double FLOWER_MID_MS = 1000;
    /** 上方一排吸球停留时间（毫秒）。 */
    public static double FLOWER_TOP_MS = 1000;
    /** 装填区停留时间（毫秒）。 */
    public static double LOAD_BOT_MS = 500;
    /** 到射球点后稳定多久再开门（毫秒）。 */
    public static double SETTLE_MS = 300;
    /** 第一次到 SHOOT_TOP 的开门时间（毫秒）。 */
    public static double SHOOT_TOP_GATE_MS = 1000;
    /** 第二次到 SHOOT_TOP_2 的开门时间（毫秒）。 */
    public static double SHOOT_TOP_2_GATE_MS = 1000;

    @Override
    public Pose getStartPose() {
        return START_BOT_VER.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                // 开局：自动瞄准 + shooter 加速 + transfer 一直转，原地射预装球
                startAuto(),
                waitMs(SPIN_UP_MS),
                shoot(PRELOAD_GATE_MS),

                // 中间一排
                follow(startToPrepmid()),
//                flowerDown(),
                waitMs(FLOWER_DOWN_MS),
                follow(prepmidToFlowermid()),
                waitMs(FLOWER_MID_MS),
                follow(flowermidToPrepmid()),
//                flowerUp(),
                follow(prepmidToShoottop()),
                waitMs(SETTLE_MS),
                shoot(SHOOT_TOP_GATE_MS),

                // 上方一排
                follow(shoottopToPreptop()),
//                flowerDown(),
                waitMs(FLOWER_DOWN_MS),
                follow(preptopToFlowertop()),
                waitMs(FLOWER_TOP_MS),
                shoot(1000),
//                flowerUp(),

                // 下方装填区
                follow(flowertopToPrepbot()),
                follow(prepbotToLoadbot()),
                waitMs(LOAD_BOT_MS),
                follow(loadbotToShoottop()),
                waitMs(SETTLE_MS),
                shoot(SHOOT_TOP_2_GATE_MS),

                // 停车
                follow(shoottopToPark()),
                stopAll()
        );
    }

    // ================= 路径 =================

    public Path startToPrepmid() {
        Pose a = START_BOT_VER.pose(), b = PREP_MID.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path prepmidToFlowermid() {
        Pose a = PREP_MID.pose(), b = FLOWER_MID.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path flowermidToPrepmid() {
        Pose a = FLOWER_MID.pose(), b = PREP_MID_2.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path prepmidToShoottop() {
        Pose a = PREP_MID_2.pose(), b = SHOOT_TOP.pose();
        return Paths.curve(a, CTRL_PREP_MID_TO_SHOOT_TOP.pose(), b).linear(a, b);
    }

    public Path shoottopToPreptop() {
        Pose a = SHOOT_TOP.pose(), b = PREP_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path preptopToFlowertop() {
        Pose a = PREP_TOP.pose(), b = FLOWER_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path flowertopToPrepbot() {
        Pose a = FLOWER_TOP.pose(), b = PREP_BOT.pose();
        return Paths.curve(a,CTRL_FLOWERTOP_PREPBOT_1.pose(),CTRL_FLOWERTOP_PREPBOT_2.pose(), b).linear(a, b);
    }

    public Path prepbotToLoadbot() {
        Pose a = PREP_BOT.pose(), b = LOAD_BOT.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path loadbotToShoottop() {
        Pose a = LOAD_BOT.pose(), b = SHOOT_TOP_2.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path shoottopToPark() {
        Pose a = SHOOT_TOP_2.pose(), b = PARK.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
