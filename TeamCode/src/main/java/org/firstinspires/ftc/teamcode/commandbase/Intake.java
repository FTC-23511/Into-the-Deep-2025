package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakePivotState.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorDetected.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorTarget.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState.*;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.nio.file.Watchable;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private final double divideConstant = 65.0;
    public double target;
    public boolean extendoReached;
    // Between retracted and extended
    public boolean extendoRetracted;
    public boolean pivotReached = false;
    // Between transfer and intake position
    // Whether the claw is open or not in the current state of the claw
    public enum IntakePivotState {
        INTAKE,
        INTAKE_READY,
        INTAKE_HOVER,
        TRANSFER,
        TRANSFER_READY
    }

    public enum IntakeMotorState {
        REVERSE,
        STOP,
        FORWARD,
        HOLD
    }

    public enum SampleColorTarget {
        ALLIANCE_ONLY,
        ANY_COLOR
    }

    public enum SampleColorDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    public static SampleColorDetected sampleColor = NONE;
    public static SampleColorTarget sampleColorTarget = ANY_COLOR;
    public static IntakePivotState intakePivotState = TRANSFER;
    public static IntakeMotorState intakeMotorState = STOP;
    private static final PIDFController extendoPIDF = new PIDFController(0.0245,0,0.0003, 0);

    public void init() {
        setPivot(TRANSFER);
        setExtendoTarget(0);
        extendoPIDF.setTolerance(15);
        robot.colorSensor.enableLed(true);
    }

    public void autoUpdateExtendo() {
        double extendoPower = extendoPIDF.calculate(getExtendoScaledPosition(), this.target);
        extendoReached = (extendoPIDF.atSetPoint() && target > 0) || (getExtendoScaledPosition() <= 8 && target == 0);
        extendoRetracted = (target <= 0) && extendoReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0 && !extendoReached) {
            extendoPower -= 0.2;
        } else if (!extendoReached) {
            extendoPower += 0.2;
        }

        if (extendoReached) {
            robot.extension.setPower(0);
        } else {
            robot.extension.setPower(extendoPower);
        }
    }

    public double getExtendoScaledPosition() {
        return robot.extensionEncoder.getPosition() / divideConstant;
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public void setPivot(IntakePivotState intakePivotState) {
        switch (intakePivotState) {
            case TRANSFER:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                break;

            case TRANSFER_READY:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_READY_TRANSFER_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_READY_TRANSFER_POS);
                break;

            case INTAKE:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_INTAKE_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_INTAKE_POS);
                break;

            case INTAKE_READY:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_READY_INTAKE_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_READY_INTAKE_POS);
                break;

            case INTAKE_HOVER:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_HOVER_INTAKE_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_HOVER_INTAKE_POS);
                break;
        }

        Intake.intakePivotState = intakePivotState;
    }

    public void setActiveIntake(IntakeMotorState intakeMotorState) {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    robot.intakeMotor.setPower(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                    break;
                case STOP:
                    robot.intakeMotor.setPower(0);
                    break;
            }
            Intake.intakeMotorState = intakeMotorState;
        } else if (intakeMotorState.equals(HOLD)) {
            robot.intakeMotor.setPower(INTAKE_HOLD_SPEED);
            Intake.intakeMotorState = intakeMotorState;
        }
    }

    public void toggleActiveIntake(SampleColorTarget sampleColorTarget) {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            if (intakeMotorState.equals(FORWARD)) {
                setActiveIntake(STOP);
            } else if (intakeMotorState.equals(STOP) || intakeMotorState.equals(HOLD)) {
                setActiveIntake(FORWARD);
            }
            Intake.sampleColorTarget = sampleColorTarget;
        }
    }

    public void autoUpdateActiveIntake() {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    if (hasSample()) {
                        sampleColor = sampleColorDetected(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                        if (correctSampleDetected()) {
                            setActiveIntake(STOP);
                            if (opModeType.equals(OpModeType.TELEOP)) {
                                if (sampleColorTarget.equals(ANY_COLOR)) {
                                    CommandScheduler.getInstance().schedule(
                                            new UninterruptibleCommand(
                                                    new SequentialCommandGroup(
                                                            new WaitCommand(150),
                                                            new RealTransfer(robot)
                                                    )
                                            )
                                    );
                                } else {
                                    CommandScheduler.getInstance().schedule(
                                            new SetIntake(robot, TRANSFER_READY, HOLD, 0, false)
                                    );
                                }
                            }
                        } else if (!sampleColor.equals(NONE)) {
                            setActiveIntake(REVERSE);
                        }
                    } else {
                        sampleColor = NONE;
                    }
                    break;
                case REVERSE:
                    if (!hasSample()) {
                        if (opModeType.equals(OpModeType.TELEOP)) {
                            setActiveIntake(FORWARD);
                        } else {
                            setActiveIntake(STOP);
                        }
                    }
                    break;

                // No point of setting intakeMotor to 0 again
            }
        } else if (intakePivotState.equals(TRANSFER) || intakePivotState.equals(TRANSFER_READY)) {
            setActiveIntake(HOLD);
        }
    }

    public static SampleColorDetected sampleColorDetected(int red, int green, int blue) {
            if (blue >= green && blue >= red) {
                return BLUE;
            } else if (green >= red) {
                return YELLOW;
            } else {
                return RED;
            }
    }

    public static boolean correctSampleDetected() {
        switch (sampleColorTarget) {
            case ANY_COLOR:
                if (sampleColor.equals(YELLOW) ||
                   (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                    sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))) {
                    return true;
                }
                break;
            case ALLIANCE_ONLY:
                if (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                    sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED)) {
                    return true;
                }
                break;
        }
        return false;
    }

    public boolean hasSample() {
        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        SampleColorDetected sampleColor = sampleColorDetected(red, green, blue);

        // For edge case intake - currently set to always true
        boolean colorSensingHasSample = true;
        switch (sampleColor) {
            case YELLOW:
                if (green > YELLOW_THRESHOLD) {
                    colorSensingHasSample = true;
                }
                break;
            case RED:
                if (red > RED_THRESHOLD) {
                    colorSensingHasSample = true;
                }
                break;
            case BLUE:
                if (blue > BLUE_THRESHOLD) {
                    colorSensingHasSample = true;
                }
                break;
        }

        return distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD && colorSensingHasSample;
    }

    @Override
    public void periodic() {
        autoUpdateExtendo();
        autoUpdateActiveIntake();
    }
}
