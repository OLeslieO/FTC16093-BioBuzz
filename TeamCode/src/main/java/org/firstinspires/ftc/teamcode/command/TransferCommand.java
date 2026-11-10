package org.firstinspires.ftc.teamcode.command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;

/**
 * 按住期间让 transfer 保持指定状态，松开（命令结束或被打断）后回到 STOP。
 *
 * <p>配合 {@code whileHeld} 使用；电机功率由 {@link TransferSubsystem#periodic()} 每个循环写出。
 */
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
