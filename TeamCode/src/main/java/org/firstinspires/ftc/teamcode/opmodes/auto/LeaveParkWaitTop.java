package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.*;
import static com.pedropathing.ivy.commands.Commands.waitMs;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoCommandBase;


@Autonomous(name = "LeaveParkWait", group = "main")
@Configurable
public class LeaveParkWaitTop extends AutoCommandBase {


    @Override
    public Pose getStartPose() {
        return LEAVE_START.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                waitMs(24000),
                follow(path1())
        );
    }


    public Path path1() {
        Pose a = LEAVE_START.pose(), b = LEAVE_END.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
