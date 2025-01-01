package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.seattlesolvers.ftclib.command.ParallelCommandGroup;
import com.seattlesolvers.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class RealTransfer extends SequentialCommandGroup {

    public RealTransfer(Robot robot) {
        addCommands(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, SLIDES_PIVOT_READY_EXTENSION + 50, true),
                        new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, false)
                ),
                new Transfer(robot)
        ));
    }
}
