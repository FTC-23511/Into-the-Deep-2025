package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Constants.BACKDROP_INCREMENTAL_HEIGHT;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class depositBackdrop extends CommandBase {
    Deposit deposit;

    public depositBackdrop(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(deposit.pixelHeight * BACKDROP_INCREMENTAL_HEIGHT);
        deposit.setClaw(false, false);
        deposit.updateWrist(4);
        deposit.setArmTransfer(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
