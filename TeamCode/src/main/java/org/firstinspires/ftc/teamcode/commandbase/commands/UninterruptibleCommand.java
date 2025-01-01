package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.ftclib.command.Command;
import com.seattlesolvers.ftclib.command.CommandBase;
import com.seattlesolvers.ftclib.command.CommandScheduler;

public class UninterruptibleCommand extends CommandBase {
    private final Command command;

    public UninterruptibleCommand(Command command) {
        this.command = command;
    }

    @Override
    public void initialize() {
        command.schedule(false);
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(command);
    }
}
