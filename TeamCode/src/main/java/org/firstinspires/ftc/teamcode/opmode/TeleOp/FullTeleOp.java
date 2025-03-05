package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;
    public ElapsedTime gameTimer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;
    private boolean frontSpecimenScoring = false;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;
        depositInit = DepositPivotState.MIDDLE_HOLD;

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
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> robot.intake.toggleActiveIntake(SampleColorTarget.ANY_COLOR))
        );

        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> robot.intake.toggleActiveIntake(SampleColorTarget.ALLIANCE_ONLY))
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new SetIntake(robot, IntakePivotState.INTAKE_READY, IntakeMotorState.HOLD, MAX_EXTENDO_EXTENSION/2, true)
        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new SetIntake(robot, IntakePivotState.INTAKE_READY, intakeMotorState, MAX_EXTENDO_EXTENSION, true)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.follower.setPose(new Pose(0, 0, 0)))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.INTAKE_READY)),
                        new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.HOLD))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> robot.intake.setExtendoTarget(0))
        );

        // TO-DO: need to make into 1 method in Drive.java
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.OUT)),
                        new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.IN)),
                        () -> Drive.subPusherState.equals(Drive.SubPusherState.IN))
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.TRANSFER))
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.INTAKE))
        );

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

        // Reset CommandScheduler + make slides reached true
        driver.getGamepadButton(GamepadKeys.Button.PS).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(super::reset),
                                new InstantCommand(() -> robot.deposit.setSlideTarget(robot.deposit.getLiftScaledPosition()))
                        )
                )
        );

        // Operator Gamepad controls
        operator.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> frontSpecimenScoring = !frontSpecimenScoring)
        );

        operator.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new ConditionalCommand(
                        new UninterruptibleCommand(
                                new SetDeposit(robot, DepositPivotState.FRONT_SPECIMEN_SCORING, FRONT_HIGH_SPECIMEN_HEIGHT, false).withTimeout(1500)
                        ),
                        new UninterruptibleCommand(
                                new SetDeposit(robot, DepositPivotState.BACK_SPECIMEN_SCORING, BACK_HIGH_SPECIMEN_HEIGHT, false).withTimeout(1500)
                        ),
                        () -> frontSpecimenScoring
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new ConditionalCommand(
                        new UninterruptibleCommand(
                                new SetDeposit(robot, DepositPivotState.BACK_SPECIMEN_INTAKE, 0, false).withTimeout(1500)
                        ),
                        new UninterruptibleCommand(
                                new SetDeposit(robot, DepositPivotState.FRONT_SPECIMEN_INTAKE, 0, false).withTimeout(1500)
                        ),
                        () -> frontSpecimenScoring
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new ConditionalCommand(
                        new attachSpecimen(robot.deposit),
                        new InstantCommand(),
                        () -> depositPivotState.equals(DepositPivotState.BACK_SPECIMEN_SCORING)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1500)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SCORING, LOW_BUCKET_HEIGHT, false).withTimeout(1500)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1500)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.deposit.setClawOpen(false))
        );

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.deposit.setClawOpen(true))
        );

        operator.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.drive.setHang(Drive.HangState.RETRACT)),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, ENDGAME_ASCENT_HEIGHT, false).withTimeout(1500),
                                new WaitCommand(3000)
                        )
//                        ,
//                        new InstantCommand(() -> robot.drive.setHang(Drive.HangState.STOP)),
//                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, HIGH_BUCKET_HEIGHT, false),
//                        new InstantCommand(() -> robot.drive.setHang(Drive.HangState.RETRACT)),
//                        new WaitCommand(500),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(() -> robot.drive.setHang(Drive.HangState.EXTEND)),
//                                        new InstantCommand(() -> robot.drive.setHang(Drive.HangState.STOP)).beforeStarting(new WaitCommand(3000))
//                                ),
//                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, false)
//                        )
                )
        );

        // Hang
        operator.getGamepadButton(GamepadKeys.Button.PS).whenPressed(
                new InstantCommand(() -> robot.drive.setHang(Drive.HangState.EXTEND))
        );

        /* Untested xTeleOp Spec Automation
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.follower.setStartingPose(new Pose(6.25, 30, Math.toRadians(180)))),
                                new FollowPathCommand(robot.follower,
                                        robot.follower.pathBuilder()
                                                .addPath(
                                                        new BezierLine(
                                                                new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                                                new Point(45.757, 69.084, Point.CARTESIAN)
                                                        )
                                                )
                                                .setConstantHeadingInterpolation(Math.toRadians(0)).build(),
                                        true
                                )
                        )
                ).andThen(
                        new InstantCommand(() -> robot.follower.startTeleopDrive())
                )
        );
        */

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
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // purple is back (default) spec scoring, green is front spec scoring
        if (frontSpecimenScoring) {
            gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(1, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // Pinpoint Field Centric Code
        double speedMultiplier = 0.35 + (1 - 0.35) * gamepad1.left_trigger;
        robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier, -gamepad1.right_stick_x * speedMultiplier * 0.75, false);
        robot.follower.update();

        // Manual control of extendo
        if (gamepad1.right_trigger > 0.01 &&
            !depositPivotState.equals(DepositPivotState.TRANSFER) &&
            robot.intake.getExtendoScaledPosition() <= (MAX_EXTENDO_EXTENSION - 5)) {

            robot.intake.target += 5;
        }

        // Hang
        if (gamepad2.left_trigger > 0.5) {
            robot.drive.setHang(Drive.HangState.STOP);
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("autoEndPose", autoEndPose.toString());
        telemetryData.addData("extendoReached", robot.intake.extendoReached);
        telemetryData.addData("extendoRetracted", robot.intake.extendoRetracted);
        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("opModeType", opModeType.name());

        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("Intake sampleColor", Intake.sampleColor);
        telemetryData.addData("correctSampleDetected", Intake.correctSampleDetected());
        telemetryData.addData("intakeMotorState", Intake.intakeMotorState);

        telemetryData.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());
        telemetryData.addData("extension.getPower()", robot.extension.getPower());

        telemetryData.addData("getExtendoScaledPosition()", robot.intake.getExtendoScaledPosition());
        telemetryData.addData("getLiftScaledPosition()", robot.deposit.getLiftScaledPosition());

        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("extendo target", robot.intake.target);

        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("Sigma", "Oscar");

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}