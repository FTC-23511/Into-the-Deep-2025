package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class ServoOnlyTransfer extends CommandBase {
    private final Robot robot;
    private boolean finished = false;
    private int index = 0;
    private ElapsedTime timer;

    public ServoOnlyTransfer(Robot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime();

        addRequirements(robot.intake, robot.deposit);
    }

    @Override
    public void initialize() {
        robot.deposit.setPivot(Deposit.DepositPivotState.READY_TRANSFER);
        robot.deposit.setClawOpen(true);

        timer.reset();
        index = 1;
    }

    @Override
    public void execute() {
        if (index == 1 && timer.milliseconds() > 300) {
            robot.deposit.setPivot(Deposit.DepositPivotState.TRANSFER);

            timer.reset();
            index = 2;
        } else if (index == 2 && timer.milliseconds() > 200) {
            robot.deposit.setClawOpen(false);

            timer.reset();
            index = 3;
        } else if (index == 3 && timer.milliseconds() > 200) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}