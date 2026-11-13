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
import org.firstinspires.ftc.teamcode.subsystem.NectarGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PollenGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

/**
 * 手动阶段：底盘 robot-centric 驾驶。
 *
 * <p>左摇杆平移，右摇杆左右转向；按住 dpad 左键进入慢速模式。
 * <p>Intake：A 伸出，B 中间，X 收回。
 * <p>Transfer：按住右肩键 intake，按住左肩键 outake，松开停止。
 * <p>Nectar gate：右扳机按下打开（SHOOT），松开关闭（STOP）。
 * <p>Pollen gate：左扳机按下打开（SHOOT），松开关闭（STOP）。
 * <p>Nectar shooter：默认一直 ACCELERATE（前馈 + PID 自己控速）。
 */
@TeleOp(name = "Drive TeleOp", group = "main")
@Configurable
public class DriveTeleOp extends CommandOpModeEx {

    /** 慢速模式下的功率系数。 */
    public static double SLOW_SCALE = 0.35;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private NectarGateSubsystem nectarGate;
    private PollenGateSubsystem pollenGate;
    private NectarShooterSubsystem shooter;
    private GamepadEx driver;
    private TelemetryManager panelsTelemetry;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        nectarGate = new NectarGateSubsystem(hardwareMap);
        pollenGate = new PollenGateSubsystem(hardwareMap);
        shooter = new NectarShooterSubsystem(hardwareMap);
        shooter.accelerate();
        register(drive, intake, transfer, nectarGate, pollenGate, shooter);

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

        // 右扳机按下开 nectar 门，松开关门
        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whileActiveOnce(new StartEndCommand(nectarGate::shoot, nectarGate::stop, nectarGate));
        // 左扳机按下开 pollen 门，松开关门
        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whileActiveOnce(new StartEndCommand(pollenGate::shoot, pollenGate::stop, pollenGate));
    }

    @Override
    public void onStart() {
    }

    @Override
    public void run() {
        super.run();

        panelsTelemetry.addData("transfer state", transfer.getState());
        panelsTelemetry.addData("nectar gate state", nectarGate.getState());
        panelsTelemetry.addData("pollen gate state", pollenGate.getState());
        panelsTelemetry.addData("shooter state", shooter.getState());
        panelsTelemetry.addData("shooter target", shooter.getTargetVelocity());
        panelsTelemetry.addData("shooter velocity", shooter.getVelocity());
        panelsTelemetry.addData("shooter error", shooter.getError());
        panelsTelemetry.update(telemetry);
    }
}
