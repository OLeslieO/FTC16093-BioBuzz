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


@Autonomous(name = "RedFullPark 2", group = "main")
@Configurable
public class RedFullPark extends AutoCommandBase {

    /** 开局 shooter 加速多久再开门（毫秒）。 */
    public static double SPIN_UP_MS = 1000;
    /** 开局预装球开门时间（毫秒）。 */
    public static double PRELOAD_GATE_MS = 2500;
    /** 到 SHOOT2 后开门时间（毫秒）。 */
    public static double SHOOT2_GATE_MS = 2200;
    /** 到 SHOOT3 后开门时间（毫秒）。 */
    public static double SHOOT3_GATE_MS = 2200;

    // 点位都在 AutoConstants 里

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
