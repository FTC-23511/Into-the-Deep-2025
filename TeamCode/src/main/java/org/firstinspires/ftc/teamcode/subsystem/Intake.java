package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    // 0 is fully retracted, 1-5 is how many pixels
    public int stackHeight = 0;
    private double target;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    public boolean trayTransfer;
    public enum IntakeState {
        ON,
        OFF,
        REVERSED_ON
    }
    public IntakeState intakeState;
    private static final PIDFController extendoPIDF = new PIDFController(0,0,0, 0);

    // Loop number for when distance sensor was polled
    private int loopNumber = 0;

    public void init() {
        setIntake(IntakeState.OFF);
        setExtendoTarget(0);
    }

    public void autoExtendoSetPower() {
        double extendoPower = extendoPIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftLeft.setPower(extendoPower);
        robot.liftRight.setPower(-extendoPower);

        // Extendo is only retracted it has reached a target of 0
        extendoRetracted = ((target <= 0)) && (this.reached());
    }

    // Returns if extendo has reached the target
    public boolean reached() {
        return (extendoPIDF.atSetPoint());
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public void setIntake(IntakeState intakeState) {
        this.intakeState = intakeState;

        switch (intakeState) {
            case ON:
                robot.intakeMotor.setPower(INTAKE_POWER);
                break;
            case OFF:
                robot.intakeMotor.setPower(0);
                break;
            case REVERSED_ON:
                robot.intakeMotor.setPower(INTAKE_REVERSE_POWER);
                break;
        }
    }

    public void setPitchingIntake(int stackHeight) {
        if (stackHeight != this.stackHeight) {
            // Limit integers to 0-5
            this.stackHeight = Math.max(Math.min(stackHeight, 5), 0);
            robot.pitchingIntake.setPosition(STACK_HEIGHTS[this.stackHeight]);
        }
    }

    public void setTray(boolean trayTransfer) {
        if (trayTransfer != this.trayTransfer) {
            robot.tray.setPosition(trayTransfer ? TRAY_TRANSFER : TRAY_INTAKE);
            this.trayTransfer = trayTransfer;
        }
    }

    public boolean sampleDetected() {
        if (intakeState == IntakeState.ON) {
            if (loopNumber > INTAKE_DISTANCE_SENSOR_POLLING) {
                return robot.colorSensor.getDistance(DistanceUnit.CM) < PIXEL_DISTANCE;
            } else {
                loopNumber++;
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        autoExtendoSetPower();
    }
}
