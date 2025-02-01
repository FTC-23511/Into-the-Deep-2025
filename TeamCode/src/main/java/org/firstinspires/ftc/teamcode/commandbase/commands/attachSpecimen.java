package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;

public class attachSpecimen extends CommandBase {
    Deposit deposit;
    ElapsedTime timer;

    public attachSpecimen(Deposit deposit) {
        this.deposit = deposit;
        this.timer = new ElapsedTime();
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(BACK_HIGH_SPECIMEN_ATTACH_HEIGHT);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && timer.milliseconds() >= 400;
    }

//    @Override
//    public void end(boolean interruptable) {
//        deposit.setClawOpen(true);
//    }
}
