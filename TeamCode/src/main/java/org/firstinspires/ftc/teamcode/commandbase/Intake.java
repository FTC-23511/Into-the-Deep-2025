package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakePivotState.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorDetected.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorTarget.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.nio.file.Watchable;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final ElapsedTime reverseIntakeTimer = new ElapsedTime();
    private boolean waitingForReverse = false;
    private final ElapsedTime colorDetectionTimer = new ElapsedTime();
    private boolean readyForColorDetection = false;

    private final double divideConstant = 65.0;
    public double target;
    public boolean extendoReached;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    // Whether the claw is open or not in the current state of the claw
    public enum IntakePivotState {
        INTAKE,
        INTAKE_READY,
        TRANSFER,
        TRANSFER_READY,
        HOVER
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
        extendoReached = (extendoPIDF.atSetPoint() && target > 0) || (getExtendoScaledPosition() <= 3 && target == 0);
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

            case HOVER:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_HOVER_INTAKE_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_HOVER_INTAKE_POS);
                break;
        }

        Intake.intakePivotState = intakePivotState;
    }

    public void setActiveIntake(IntakeMotorState intakeMotorState) {
        if (intakeMotorState.equals(HOLD)) {
            robot.intakeMotor.setPower(INTAKE_HOLD_SPEED);
            Intake.intakeMotorState = intakeMotorState;
        } else if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    robot.intakeMotor.setPower(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                    reverseIntakeTimer.reset();
                    break;
                case STOP:
                    robot.intakeMotor.setPower(0);
                    break;
            }
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
                        if (!readyForColorDetection) {
                            colorDetectionTimer.reset();
                            readyForColorDetection = true;
                        } else if (readyForColorDetection && colorDetectionTimer.milliseconds() > 50) {
                            readyForColorDetection = false;

                            sampleColor = sampleColorDetected(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                            if (correctSampleDetected()) {
                                setActiveIntake(HOLD);
                                if (opModeType.equals(OpModeType.TELEOP)) {
                                    if (sampleColorTarget.equals(ANY_COLOR)) {
                                        new RealTransfer(robot).beforeStarting(
                                                new WaitCommand(125)
                                        ).schedule(false);
                                    } else {
                                        new SetIntake(robot, TRANSFER_READY, HOLD, 0, false).schedule(false);
                                    }
                                }
                            } else {
                                reverseIntakeTimer.reset();
                                setActiveIntake(REVERSE);
                            }
                        }
                    }
                    break;
                case REVERSE:
                    if (!hasSample() && !waitingForReverse) {
                        reverseIntakeTimer.reset();
                        waitingForReverse = true;
                    } else if (!hasSample() && waitingForReverse && reverseIntakeTimer.milliseconds() > REVERSE_TIME_MS) {
                        waitingForReverse = false;
                        if (opModeType.equals(OpModeType.TELEOP)) {
                            setActiveIntake(FORWARD);
                        } else {
                            setActiveIntake(STOP);
                        }
                    }
                    break;
                case HOLD:
                    if (!correctSampleDetected() && hasSample() && Intake.intakePivotState.equals(INTAKE)) {
                        setActiveIntake(REVERSE);
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
                   (sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED)))) {
                    return true;
                }
                break;
            case ALLIANCE_ONLY:
                if (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                   (sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))) {
                    return true;
                }
                break;
        }
        return false;
    }
    public boolean hasSample() {
        /* Color thresholding (not used)
        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        SampleColorDetected sampleColor = sampleColorDetected(red, green, blue);

        switch (sampleColor) {
            case YELLOW:
                if (green > YELLOW_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
            case RED:
                if (red > RED_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
            case BLUE:
                if (blue > BLUE_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
        }
         */

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        return distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD;
    }

    @Override
    public void periodic() {
        autoUpdateExtendo();
        autoUpdateActiveIntake();
    }
}
