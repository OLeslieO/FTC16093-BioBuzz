package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PollenGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

/**
 * 手动阶段：底盘 robot-centric 驾驶。
 *
 * <p>左摇杆平移，右摇杆左右转向；按住 dpad 左键进入慢速模式。
 * <p>Intake：A 伸出，B 中间，X 收回。
 * <p>Transfer：按住右肩键 intake，按住左肩键 outake，松开停止。
 * <p>Pollen gate：右扳机按下打开（SHOOT），松开关闭（STOP）。
 */
@TeleOp(name = "Drive TeleOp", group = "main")
@Configurable
public class DriveTeleOp extends CommandOpModeEx {

    /** 慢速模式下的功率系数。 */
    public static double SLOW_SCALE = 0.35;

    /** 右扳机超过这个值就打开 pollen gate。 */
    public static double GATE_TRIGGER_THRESHOLD = 0.5;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private PollenGateSubsystem gate;
    private GamepadEx driver;
    private TelemetryManager panelsTelemetry;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        gate = new PollenGateSubsystem(hardwareMap);
        register(drive, intake, transfer, gate);

        driver = new GamepadEx(gamepad1);

        // 摇杆 y 向前推是负值，所以取反
        drive.setDefaultCommand(new DriveCommand(
                drive,
                () -> gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.dpad_left ? SLOW_SCALE : 1.0
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

        // 右扳机按下开门，松开关门
        new Trigger(() -> gamepad1.right_trigger > GATE_TRIGGER_THRESHOLD)
                .whileActiveOnce(new StartEndCommand(gate::shoot, gate::stop, gate));
    }

    @Override
    public void onStart() {
    }

    @Override
    public void run() {
        super.run();

        panelsTelemetry.addData("slow mode", gamepad1.dpad_left);
        panelsTelemetry.addData("left stick", gamepad1.left_stick_x + ", " + (-gamepad1.left_stick_y));
        panelsTelemetry.addData("right stick x", gamepad1.right_stick_x);
        panelsTelemetry.addData("intake state", intake.getState());
        panelsTelemetry.addData("intake target", intake.getTargetPosition());
        panelsTelemetry.addData("transfer state", transfer.getState());
        panelsTelemetry.addData("gate state", gate.getState());
        panelsTelemetry.update(telemetry);
    }
}
