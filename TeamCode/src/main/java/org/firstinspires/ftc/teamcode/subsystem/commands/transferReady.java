package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transferReady extends ParallelCommandGroup {
    public transferReady(Deposit deposit, Intake intake) {
        addCommands(
                new depositTransferReady(deposit),
                new intakeTransferReady(intake)
        );
        addRequirements(deposit, intake);
    }
}
