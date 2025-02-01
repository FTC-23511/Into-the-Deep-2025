package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SetRobot extends ParallelCommandGroup {
    private final Robot robot;
    private final Robot.RobotState robotState;
    private boolean didItReallyHappen = false;

    public SetRobot(Robot robot, Robot.RobotState robotState) {
        this.robot = robot;
        this.robotState = robotState;

        switch (robotState) {
            case MIDDLE_RESTING:
                if (!Robot.robotState.equals(Robot.RobotState.TRANSFERRED)) {
                    addCommands(
                            new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true),
                            new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, true)
                    );
//                    addRequirements(robot.intake, robot.deposit);
                } else {
                    addCommands(
                            new UninterruptibleCommand(new UndoTransfer(robot))
                    );
//                    addRequirements(robot.intake, robot.deposit);
                }

                didItReallyHappen = true;
                break;

            case TRANSFERRED:
                if (Robot.robotState.equals(Robot.RobotState.MIDDLE_RESTING)) {
                    addCommands(
//                            new Transfer(robot)
                    );
//                    addRequirements(robot.intake, robot.deposit);

                    didItReallyHappen = true;
                }
                break;
            case SPECIMEN_SCORING:
                if (Robot.robotState.equals(Robot.RobotState.SPECIMEN_INTAKING)) {
                    addCommands(
                            new SetDeposit(robot, Deposit.DepositPivotState.FRONT_SPECIMEN_SCORING, FRONT_HIGH_SPECIMEN_HEIGHT, false)
                    );
                    addRequirements(robot.deposit);

                    didItReallyHappen = true;
                }
                break;
            case SPECIMEN_INTAKING:
                break;
        }
    }



    @Override
    public void end(boolean interrupted) {
        if (didItReallyHappen && !interrupted) {
            Robot.robotState = robotState;
        }

        super.end(interrupted);
    }

}
