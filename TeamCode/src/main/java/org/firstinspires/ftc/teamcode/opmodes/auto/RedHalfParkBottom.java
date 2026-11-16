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


@Autonomous(name = "RedHalfParkBottom", group = "main")
@Configurable
public class RedHalfParkBottom extends AutoCommandBase {

    /** 开局 shooter 加速多久再出发（毫秒）。 */
    public static double SPIN_UP_MS = 500;
    /** 到 SHOOT1 后开门时间（毫秒）。 */
    public static double SHOOT1_GATE_MS = 2500;
    /** 到 SHOOT2 后开门时间（毫秒）。 */
    public static double SHOOT2_GATE_MS = 2500;
    /** 到 SHOOT3 后开门时间（毫秒）。 */
    public static double SHOOT3_GATE_MS = 2500;

    @Override
    public Pose getStartPose() {
        return START_BOTTOM.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                startShooting(),
                waitMs(SPIN_UP_MS),
                follow(startToShoot1()),
                shoot(SHOOT1_GATE_MS),

                follow(shoot1ToPollenPrep1()),
                follow(pollenPrep1ToPollen1()),
                waitMs(400),
                follow(pollen1ToShoot2()),
                shoot(SHOOT2_GATE_MS),

                follow(shoot2ToPollenPrep2()),
                flowerDown(),
                waitMs(300),
                follow(pollenPrep2ToPollen2()),
                waitMs(1000),
                follow(pollen2ToShoot3()),
                shoot(SHOOT3_GATE_MS),

                follow(park())
        );
    }


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

    public Path pollen1ToShoot2() {
        Pose a = POLLEN_MID.pose(), b = SHOOT_BOTTOM.pose(180);
        return Paths.curve(a, CTRL_MID_TO_SHOOT_BOTTOM.pose(), b).linear(a, b);
    }

    public Path shoot2ToPollenPrep2() {
        Pose a = SHOOT_BOTTOM.pose(180), b = POLLEN_BOTTOM_PREP.pose();
        return Paths.line(a, b).linear(a, b);
    }

    public Path pollenPrep2ToPollen2() {
        return Paths.line(POLLEN_BOTTOM_PREP.pose(), POLLEN_BOTTOM.pose()).constant(POLLEN_BOTTOM.pose());
    }

    public Path pollen2ToShoot3() {
        Pose a = POLLEN_BOTTOM.pose(), b = SHOOT_BOTTOM.pose(180);
        return Paths.line(a, b).linear(a, b);
    }

    public Path park() {
        Pose a = SHOOT_BOTTOM.pose(180), b = PARK_MID.pose();
        return Paths.curve(a, CTRL_PARK_MID_1.pose(), CTRL_PARK_MID_2.pose(), b).linear(a, b);
    }
}
