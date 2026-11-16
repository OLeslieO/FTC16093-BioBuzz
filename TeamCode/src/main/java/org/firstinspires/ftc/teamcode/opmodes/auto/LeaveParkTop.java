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

/**
 * 自动阶段：按 Pedro Pathing Visualizer 导出的路线跑 path1（start → path1 直线）。
 * 子系统、循环、位姿传递都在 {@link AutoCommandBase} 里。
 *
 * <p><b>INIT 时云台要朝正前方</b>（会把当前位置当作零点）。
 */
@Autonomous(name = "LeavePark", group = "main")
@Configurable
public class LeaveParkTop extends AutoCommandBase {

    // 点位都在 AutoConstants 里

    @Override
    public Pose getStartPose() {
        return LEAVE_START.pose();
    }

    @Override
    public Command runAutoCommand() {
        return sequential(
                startShooting(),
                follow(path1()),
                stopTransfer()
        );
    }


    public Path path1() {
        Pose a = LEAVE_START.pose(), b = LEAVE_END.pose();
        return Paths.line(a, b).linear(a, b);
    }
}
