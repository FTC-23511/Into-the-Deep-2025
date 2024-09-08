package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class depositTransferReady extends CommandBase {
    Deposit deposit;
    ElapsedTime timer = new ElapsedTime();

    public depositTransferReady(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(0);
        deposit.setClaw(true, true);
        deposit.setWristTransfer();
        deposit.setArmTransfer(true);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        // Slides must be retracted and time must be given for servos to reach transfer position
        return ((timer.milliseconds() > 2000) && (deposit.slidesRetracted));
    }
}
