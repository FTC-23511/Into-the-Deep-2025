package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transfer extends SequentialCommandGroup {
    public transfer (Deposit deposit, Intake intake) {
        addCommands(
            new transferReady(deposit, intake),
            new realTransfer(deposit, intake)
        );

        addRequirements(deposit, intake);
    }
}
