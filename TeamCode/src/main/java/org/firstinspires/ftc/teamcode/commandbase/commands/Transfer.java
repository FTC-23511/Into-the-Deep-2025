package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.ftclib.command.InstantCommand;
import com.seattlesolvers.ftclib.command.SequentialCommandGroup;
import com.seattlesolvers.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Transfer extends SequentialCommandGroup {
    public Transfer(Robot robot) {
        addCommands(
                new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, 0, true),
                new WaitCommand(300),
                new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                new WaitCommand(200)
        );
        addRequirements(robot.intake, robot.deposit);
    }
}