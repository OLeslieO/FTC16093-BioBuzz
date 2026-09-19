package org.firstinspires.ftc.teamcode.command;


import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rx;
    private final DoubleSupplier speedMultiplier;

    public DriveCommand(DriveSubsystem subsystem,
                        DoubleSupplier x,
                        DoubleSupplier y,
                        DoubleSupplier rx,
                        DoubleSupplier speedMultiplier) {
        driveSubsystem = subsystem;
        this.x = x;
        this.y = y;
        this.rx = rx;
        this.speedMultiplier = speedMultiplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.setBotCentric(
                x.getAsDouble(),
                y.getAsDouble(),
                rx.getAsDouble(),
                speedMultiplier.getAsDouble()
        );
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

}
