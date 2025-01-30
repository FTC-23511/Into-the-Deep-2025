package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class RescheduleScoring extends SequentialCommandGroup {
    public RescheduleScoring(Robot robot, Command command, boolean condition) {
        addCommands(
                    command,
                    new RepeatCommand(
                            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new RealTransfer(robot),
                                            command
                                    ),
                                    new InstantCommand(),
                                    () -> condition
                            )
                    ).interruptOn(() -> !condition)
        );
        addRequirements(robot.intake, robot.deposit);
    }
}