package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoCommandBase;
import static com.pedropathing.ivy.commands.Commands.waitMs;



@Autonomous(name = "LeaveShootPark", group = "main")
@Configurable
public class LeaveShootParkTop extends AutoCommandBase {


    @Override
    public Pose getStartPose() {
        return LEAVE_START.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                startShooting(),
                follow(path1()),
                waitMs(500),
                shoot(3000),
                follow(path2()),
                stopAll()

        );
    }

    // ================= 路径（点位见 AutoConstants） =================

    public Path path1() {
        Pose a = LEAVE_START.pose(), b = SHOOT_TOP.pose();
        return Paths.line(a, b).linear(a, b);
    }
    public Path path2() {
        Pose a = SHOOT_TOP.pose(), b = LEAVE_END.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
