package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

/**
 * 手动阶段：底盘 robot-centric 驾驶。
 *
 * <p>左摇杆平移，右摇杆左右转向；按住右扳机进入慢速模式。
 */
@TeleOp(name = "Drive TeleOp", group = "main")
@Configurable
public class DriveTeleOp extends CommandOpModeEx {

    /** 慢速模式下的功率系数。 */
    public static double SLOW_SCALE = 0.35;

    /** 右扳机超过这个值就算按下。 */
    public static double SLOW_TRIGGER_THRESHOLD = 0.5;

    private DriveSubsystem drive;
    private TelemetryManager panelsTelemetry;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new DriveSubsystem(hardwareMap);
        register(drive);

        // 摇杆 y 向前推是负值，所以取反
        drive.setDefaultCommand(new DriveCommand(
                drive,
                () -> gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.right_trigger > SLOW_TRIGGER_THRESHOLD ? SLOW_SCALE : 1.0
        ));
    }

    @Override
    public void functionalButtons() {
    }

    @Override
    public void onStart() {
    }

    @Override
    public void run() {
        super.run();

        panelsTelemetry.addData("slow mode", gamepad1.right_trigger > SLOW_TRIGGER_THRESHOLD);
        panelsTelemetry.addData("left stick", gamepad1.left_stick_x + ", " + (-gamepad1.left_stick_y));
        panelsTelemetry.addData("right stick x", gamepad1.right_stick_x);
        panelsTelemetry.update(telemetry);
    }
}
