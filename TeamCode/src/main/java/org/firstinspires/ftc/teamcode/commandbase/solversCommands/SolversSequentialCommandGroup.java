package org.firstinspires.ftc.teamcode.commandbase.solversCommands;

import com.arcrobotics.ftclib.command.Command;

import java.util.ArrayList;
import java.util.List;

public class SolversSequentialCommandGroup extends SolversCommandGroupBase {

    private final List<Command> m_commands = new ArrayList<>();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;

    /**
     * Creates a new SequentialCommandGroup.  The given commands will be run sequentially, with
     * the CommandGroup finishing when the last command finishes.
     *
     * @param commands the commands to include in this group.
     */
    public SolversSequentialCommandGroup(Command... commands) {
        addCommands(commands);
    }


    @Override
    public void addCommands(Command... commands) {
        requireUngrouped(commands);

        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

        registerGroupedCommands(commands);

        for (Command command : commands) {
            m_commands.add(command);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        m_currentCommandIndex = 0;

        if (!m_commands.isEmpty()) {
            m_commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (m_commands.isEmpty()) {
            return;
        }
        if (m_currentCommandIndex == -1) {
            return;
        }

        Command currentCommand = m_commands.get(m_currentCommandIndex);

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            m_currentCommandIndex++;
            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_currentCommandIndex == -1) {
            return;
        }
        if (interrupted && !m_commands.isEmpty()) {
            m_commands.get(m_currentCommandIndex).end(true);
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

}