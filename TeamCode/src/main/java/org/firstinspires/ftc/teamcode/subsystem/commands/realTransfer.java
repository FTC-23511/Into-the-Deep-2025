package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class realTransfer extends CommandBase {
    Deposit deposit;
    Intake intake;
    ElapsedTime timer = new ElapsedTime();
    boolean trayReached = false;

    public realTransfer(Deposit deposit, Intake intake) {
        this.deposit = deposit;
        this.intake = intake;
        addRequirements(deposit, intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        intake.setTray(true);
    }

    @Override
    public void execute() {
        if (timer.milliseconds() > 500) {
            deposit.setClaw(false, false);
            timer.reset();
            trayReached = true;
        }
    }

    // Command is finished once the tray has reached, and an extra 100ms have passed for claw to grip pixels
    @Override
    public boolean isFinished() {
        return (trayReached && (timer.milliseconds() > 100));
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTray(false);
    }
}
