package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SetDeposit extends CommandBase {
    private final Robot robot;
    private final Deposit.DepositPivotState pivotState;
    private final double target;
    private final boolean clawOpen;

    ElapsedTime timer;
    private double index;
    private double previousServoPos;
    private double currentServoPos;

    public SetDeposit(Robot robot, Deposit.DepositPivotState pivotState, double target, boolean clawOpen) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.target = target;
        this.clawOpen = clawOpen;
        this.timer = new ElapsedTime();

        addRequirements(robot.deposit);
    }

    @Override
    public void initialize() {

        if (Deposit.depositPivotState.equals(this.pivotState) && robot.deposit.target == this.target) {
            index = 3;
        } else {
            // Always close claw first in case of any arm movements that need to be done
            robot.deposit.setClawOpen(false);

            // Move slides to above pivot ready extension if target is below the pivot ready extension so that arm can move later
            // If it is more than that just yolo it because slides are faster than the pivot so arm is ready to move instantly
            if (target >= SLIDES_PIVOT_READY_EXTENSION) {
                robot.deposit.setSlideTarget(target);

                // Index for moving the arm
                if (pivotState.equals(Deposit.DepositPivotState.FRONT_SPECIMEN_SCORING) || pivotState.equals(Deposit.DepositPivotState.BACK_SPECIMEN_SCORING)) {
                    index = 0.5;
                } else {
                    index = 1;
                }
            } else {
                robot.deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);

                // Index for wait for slide move up before move arm
                index = 0;
            }

            timer.reset();
        }

    }

    @Override
    public void execute() {
        // Wait until slide is above height that pivot won't hit axle
        if (robot.liftEncoder.getPosition() >= SLIDES_PIVOT_READY_EXTENSION && index == 0) {
            index = 1;
        }

        // Wait for slides to go a bit up if going to specimen scoring so that clip doesn't hit wall
        if (index == 0.5 && timer.milliseconds() >= 200) {
            index = 1;
        }

        // Move the pivot
        if (index == 1) {
            // Update previous and current servo pivot pos for timer logic in next if statement
            previousServoPos = robot.leftDepositPivot.getPosition();

            robot.deposit.setPivot(pivotState);

            currentServoPos = robot.leftDepositPivot.getPosition();
            timer.reset();

            index = 2;
        }

        // Final move of claw and slides to the original desired state after arm has moved
        // Uses difference in pos of claw multiplied by a constant that converts servoPos change to milliseconds (DEPOSIT_PIVOT_MOVEMENT_TIME)
        if (index == 2 && timer.milliseconds() > (Math.abs(previousServoPos - currentServoPos) * DEPOSIT_PIVOT_MOVEMENT_TIME)) {
            robot.deposit.setSlideTarget(target);
            robot.deposit.setClawOpen(clawOpen);

            index = 3;
        }
    }

    // Command finishes when slides have reached and all arm movements are finished
    @Override
    public boolean isFinished() {
//        if (robot.intake.hasSample() && opModeType.equals(OpModeType.AUTO) &&
//            robot.deposit.getLiftScaledPosition() > SLIDES_PIVOT_READY_EXTENSION &&
//            pivotState.equals(Deposit.DepositPivotState.SCORING)) {
//
//            this.cancel();
//        }
        return robot.deposit.slidesReached && index == 3;
    }

//    @Override
//    public void end(boolean interrupted) {
//        if (interrupted) {
//            new SequentialCommandGroup(
//                    new RealTransfer(robot),
//                    this
//            ).schedule(false);
//        }
//    }
}

