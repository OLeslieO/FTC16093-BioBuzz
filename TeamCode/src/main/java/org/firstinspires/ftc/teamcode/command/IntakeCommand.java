package org.firstinspires.ftc.teamcode.command;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class IntakeCommand extends InstantCommand {

    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.State state) {
        super(() -> intake.setState(state), intake);
    }
}
