package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class UndoTransfer extends CommandBase {
    private final Robot robot;

    ElapsedTime timer;
    private int index;
    private double previousServoPos;
    private double currentServoPos;

    public UndoTransfer(Robot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime();

        addRequirements(robot.intake, robot.deposit);
    }

    @Override
    public void initialize() {
        robot.deposit.setClawOpen(true);
        timer.reset();

        index = 0;

        if (!Deposit.depositPivotState.equals(Deposit.DepositPivotState.TRANSFER)) {
            index = 3;
        }
    }

    @Override
    public void execute() {
        // Wait for claw to open/let go of the sample
        if (timer.milliseconds() > 200 && index == 0) {
            // Raise the slides
            robot.deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);

            index = 1;
        }

        // Move arm
        if (robot.liftEncoder.getPosition() >= SLIDES_PIVOT_READY_EXTENSION && index == 1) {
            // Close claw so that it doesn't hit slides
            robot.deposit.setPivot(Deposit.DepositPivotState.MIDDLE_HOLD);
            robot.deposit.setClawOpen(false);

            index = 2;
        }

        // Move the slides back down
        if (index == 2 && timer.milliseconds() > (Math.abs(previousServoPos - currentServoPos) * DEPOSIT_PIVOT_MOVEMENT_TIME)) {
            robot.deposit.setClawOpen(true);
            robot.deposit.setSlideTarget(0);
            index = 3;
        }
    }

    // Command finishes when all execute commands have finished (index == 3), and slides have retracted
    @Override
    public boolean isFinished() {
        return robot.deposit.slidesReached && index == 3;
    }
}

