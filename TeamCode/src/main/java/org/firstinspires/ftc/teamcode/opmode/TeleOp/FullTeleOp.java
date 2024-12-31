package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.commands.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;
    public ElapsedTime gameTimer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;

        INTAKE_HOLD_SPEED = 0;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.intake.setActiveIntake(IntakeMotorState.STOP);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> robot.intake.toggleActiveIntake(SampleColorTarget.ANY_COLOR)));

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.intake.toggleActiveIntake(SampleColorTarget.ALLIANCE_ONLY)));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> robot.follower.setPose(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), 0))));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new UninterruptibleCommand(
                        new SetIntake(robot, IntakePivotState.INTAKE_READY, intakeMotorState, MAX_EXTENDO_EXTENSION, false)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SPECIMEN_SCORING, ENDGAME_ASCENT_HEIGHT, false)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptibleCommand(
                        new ParallelCommandGroup(
                                new SetDeposit(robot, DepositPivotState.SPECIMEN_SCORING, 0, false),
                                new SetIntake(robot, intakePivotState, intakeMotorState, 0, false)

                        )
                )
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.INTAKE_READY)));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.TRANSFER)));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.INTAKE)));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new UndoTransfer(robot),
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.REVERSE, MAX_EXTENDO_EXTENSION, true)
                        )
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new RealTransfer(robot)
                )
        );

        // Operator Gamepad controls
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> robot.deposit.setClawOpen(!robot.deposit.clawOpen)));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SCORING, LOW_BUCKET_HEIGHT, false)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SPECIMEN_SCORING, HIGH_SPECIMEN_HEIGHT, false)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SPECIMEN_INTAKE, 0, true)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new UninterruptibleCommand(
                        new attachSpecimen(robot.deposit)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.TRANSFER, SLIDES_PIVOT_READY_EXTENSION + 50, true)
                )
        );

        super.run();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();

            INTAKE_HOLD_SPEED = 0.15; // Enable hold

            timer = new ElapsedTime();
            gameTimer = new ElapsedTime();
        }
        // Endgame/hang rumble after 105 seconds to notify robot.driver to hang
        else if ((gameTimer.seconds() > 105) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        if (sampleColor.equals(SampleColorDetected.RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // OTOS Field Centric Code
        double speedMultiplier = 0.35 + (1 - 0.35) * gamepad1.left_trigger;
        robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier, -gamepad1.right_stick_x * speedMultiplier, false);
        robot.follower.update();

        if (gamepad1.right_trigger > 0.01 &&
                !depositPivotState.equals(DepositPivotState.TRANSFER) &&
                robot.extensionEncoder.getPosition() <= (MAX_EXTENDO_EXTENSION - 5)) {

            robot.intake.target += 5;
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("extendoReached", robot.intake.extendoReached);
        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("robotState", Robot.robotState);

        telemetryData.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());
        telemetryData.addData("extension.getPower()", robot.extension.getPower());

        telemetryData.addData("extensionEncoder.getPosition()", robot.extensionEncoder.getPosition());
        telemetryData.addData("liftEncoder.getPosition()", robot.liftEncoder.getPosition());

        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("extendo target", robot.intake.target);

        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}