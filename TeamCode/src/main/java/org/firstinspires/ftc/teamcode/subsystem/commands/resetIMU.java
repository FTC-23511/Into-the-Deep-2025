package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class resetIMU extends CommandBase {
    private Robot robot;

    public resetIMU(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.resetIMU();
    }

}
