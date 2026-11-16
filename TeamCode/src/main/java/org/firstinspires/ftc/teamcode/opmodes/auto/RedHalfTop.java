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
