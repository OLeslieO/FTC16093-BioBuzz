package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

/**
 * 基础 TeleOp（手柄 1）：
 * <ul>
 *   <li>左摇杆平移，右摇杆 x 转向（robot-centric）</li>
 *   <li>右肩键：intake（transfer 吸球）</li>
 *   <li>左肩键：outtake（transfer 吐球）；两个都按时右肩键优先</li>
 *   <li>右扳机：按住开 gate，松开关</li>
 *   <li>shooter 一直以 SHOOTER_VEL（ticks/s）运行</li>
 * </ul>
 */
@TeleOp(name = "Basic TeleOp", group = "main")
@Configurable
public class BasicTeleOp extends CommandOpModeEx {

    public static double SHOOTER_VEL = 1200.0;
    public static double DRIVE_POWER = 1.0;
    public static double GATE_TRIGGER_THRESHOLD = 0.5;

    private DriveSubsystem drive;
    private TransferSubsystem transfer;
    private GateSubsystem gate;
    private ShooterSubsystem shooter;
    private Telemetry out;

    @Override
    public void initialize() {
        out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        drive = new DriveSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // init 时都停着，按 START 才转
        transfer.stop();
        gate.stop();
        shooter.stop();
        register(drive, transfer, gate, shooter);
    }

    @Override
    public void functionalButtons() {
    }

    @Override
    public void onStart() {
        shooter.setVelocity(SHOOTER_VEL);
    }

    @Override
    public void run() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drive.setBotCentric(x, y, rx, DRIVE_POWER);

        if (gamepad1.right_bumper) {
            transfer.setState(TransferSubsystem.State.INTAKE);
        } else if (gamepad1.left_bumper) {
            transfer.setState(TransferSubsystem.State.OUTAKE);
        } else {
            transfer.stop();
        }

        if (gamepad1.right_trigger > GATE_TRIGGER_THRESHOLD) {
            gate.shoot();
        } else {
            gate.stop();
        }

        // Panels 里改 SHOOTER_VEL 实时生效
        shooter.setVelocity(SHOOTER_VEL);

        super.run(); // 跑 scheduler → 各 subsystem 的 periodic 写硬件

        out.addData("Shooter Target", "%.0f", shooter.getTargetVelocity());
        out.addData("Shooter Vel", "%.0f", shooter.getVelocity());
        out.addData("Shooter Ready", shooter.isReady());
        out.addData("Transfer", transfer.getState());
        out.addData("Gate", gate.getState());
        out.update();
    }

    @Override
    public void end() {
        drive.stop();
    }
}
