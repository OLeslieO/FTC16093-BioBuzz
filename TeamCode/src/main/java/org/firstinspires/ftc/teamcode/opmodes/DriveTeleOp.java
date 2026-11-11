package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

/**
 * 手动阶段：底盘 robot-centric 驾驶。
 *
 * <p>左摇杆平移，右摇杆左右转向；按住右扳机进入慢速模式。
 * <p>Intake：A 伸出，B 中间，X 收回。
 * <p>Transfer：按住右肩键 intake，按住左肩键 outake，松开停止。
 */
@TeleOp(name = "Drive TeleOp", group = "main")
@Configurable
public class DriveTeleOp extends CommandOpModeEx {

    /** 慢速模式下的功率系数。 */
    public static double SLOW_SCALE = 0.35;

    /** 右扳机超过这个值就算按下。 */
    public static double SLOW_TRIGGER_THRESHOLD = 0.5;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private GamepadEx driver;
    private TelemetryManager panelsTelemetry;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        register(drive, intake, transfer);

        driver = new GamepadEx(gamepad1);

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
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeCommand(intake, IntakeSubsystem.State.EXTENDED));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new IntakeCommand(intake, IntakeSubsystem.State.MIDDLE));
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new IntakeCommand(intake, IntakeSubsystem.State.RETRACTED));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new TransferCommand(transfer, TransferSubsystem.State.INTAKE));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new TransferCommand(transfer, TransferSubsystem.State.OUTAKE));
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
        panelsTelemetry.addData("intake state", intake.getState());
        panelsTelemetry.addData("intake target", intake.getTargetPosition());
        panelsTelemetry.addData("transfer state", transfer.getState());
        panelsTelemetry.update(telemetry);
    }
}
