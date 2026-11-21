package org.firstinspires.ftc.teamcode.command;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.FlowerSubsystem;

public class IntakeCommand extends InstantCommand {

    public IntakeCommand(FlowerSubsystem intake, FlowerSubsystem.State state) {
        super(() -> intake.setState(state), intake);
    }
}
