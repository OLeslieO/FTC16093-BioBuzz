package org.firstinspires.ftc.teamcode.command;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/**
 * 把 intake 切到指定档位，立即结束。
 *
 * <p>舵机位置由 {@link IntakeSubsystem#periodic()} 每个循环写出，所以这里只改状态。
 */
public class IntakeCommand extends InstantCommand {

    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.State state) {
        super(() -> intake.setState(state), intake);
    }
}
