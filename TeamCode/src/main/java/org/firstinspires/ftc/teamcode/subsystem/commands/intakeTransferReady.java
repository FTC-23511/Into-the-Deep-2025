package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class intakeTransferReady extends CommandBase {
    Intake intake;

    public intakeTransferReady(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtendoTarget(0);
        intake.setIntake(Intake.IntakeState.OFF);
        intake.setTray(false);
        intake.setPitchingIntake(0);
    }

    @Override
    public boolean isFinished() {
        return (intake.extendoRetracted);
    }
}
