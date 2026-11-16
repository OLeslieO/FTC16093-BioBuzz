package org.firstinspires.ftc.teamcode.command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

public class TransferCommand extends CommandBase {

    private final TransferSubsystem transfer;
    private final TransferSubsystem.State state;

    public TransferCommand(TransferSubsystem transfer, TransferSubsystem.State state) {
        this.transfer = transfer;
        this.state = state;
        addRequirements(transfer);
    }

    @Override
    public void initialize() {
        transfer.setState(state);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
    }
}
