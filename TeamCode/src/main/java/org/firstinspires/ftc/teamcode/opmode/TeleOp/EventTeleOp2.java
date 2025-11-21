package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import android.annotation.SuppressLint;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

@TeleOp(name = "EventTeleOp with logging")
public class EventTeleOp2 extends CommandOpMode {
    private static final org.slf4j.Logger log = LoggerFactory.getLogger(EventTeleOp.class);
    private FileHandler logFileHandler;
    private Logger logger;
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
                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION / 2, true)
        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true)
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

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1500),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                new WaitCommand(300),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1500)
                        ),
                        () -> robot.deposit.target == 0
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.INTAKE))
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new RealTransfer(robot)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.PS).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.drive.setHang(Drive.HangState.EXTEND)),
                                new WaitCommand(3000),
                                new InstantCommand(() -> robot.drive.setHang(Drive.HangState.STOP)),
                                new InstantCommand(() -> endgame = true)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.drive.setHang(Drive.HangState.RETRACT)),
                                new SetDeposit(robot, DepositPivotState.INSIDE, ENDGAME_ASCENT_HEIGHT, false)
                        ),
                        (() -> !endgame)
                )
        );

        initLogger();

        super.run();
    }


    private boolean initLogger() {
        if (logger != null) {
            return true;
        }

        try {
            // Create a FileHandler to write logs to a file named "my_application.log"
            // The 'true' argument means to append to the file if it exists,
            // otherwise, a new file will be created.
            logFileHandler = new FileHandler("/sdcard/FIRST/test_logger.log", false);

            // Timestamp,object type,value
            // 11:54:35,Double,0.5

            // Set the formatter for the FileHandler. SimpleFormatter is a common choice.

//            SimpleFormatter formatter = new SimpleFormatter();
//            logFileHandler.setFormatter(formatter);


            Logger logging = Logger.getLogger("CustomLogger");
            ConsoleHandler handler = new ConsoleHandler();
            handler.setFormatter(new CustomFormatter());
            logging.addHandler(handler);



            logger = Logger.getLogger(EventTeleOp.class.getName());
            // Add the FileHandler to the logger.
            logger.addHandler(logFileHandler);

            // Optionally, set the logging level for the logger.
            // Messages with a level higher than or equal to this will be logged.
            logger.setLevel(Level.INFO);

            // Log some messages
            // TODO: to be removed later
//            logger.info("This is an informational message.");
//            logger.warning("A warning occurred in the application.");
//            logger.severe("A severe error has occurred!");
//            logger.log(Level.FINE, "This is a fine-grai'ned debug message."); // This won't be logged with INFO level
        } catch (SecurityException | IOException e) {
            e.printStackTrace();
            return false;
        }

        return true;
    }

    public class CustomFormatter extends Formatter {
        @Override
        public String format(LogRecord record) {

            // Convert millis → readable timestamp
            @SuppressLint("DefaultLocale") String timestamp = String.format("%1$tF %1$tT", new java.util.Date(record.getMillis()));

            // object type = log level (INFO, WARNING, etc.)
            String objectType = record.getLevel().getName();

            // value = the message
            String value = record.getMessage();

            // return as: Timestamp,object type,value
            return timestamp + "," + objectType + "," + value + "\n";
        }
        }

    private void deinitLogger() {
        logger.info("Deinitialize the logger");
        logger = null;
        logFileHandler.close();
        logFileHandler = null;
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

        if (sampleColor.equals(SampleColorDetected.RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // Control Hub IMU Field Centric Code

        double speedMultiplier = 0.35 + (0.65 * driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        double botHeading = robot.getYawDegrees();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        robot.leftFront.setPower(frontLeftPower * speedMultiplier);
        robot.leftBack.setPower(backLeftPower * speedMultiplier);
        robot.rightFront.setPower(frontRightPower * speedMultiplier);
        robot.rightBack.setPower(backRightPower * speedMultiplier);

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();
        //logging
        logger.info("timer: " + timer.milliseconds());
        logger.info("autoEndPose: " + autoEndPose.toString());
        logger.info("extendoReached: " + robot.intake.extendoReached);
        logger.info("extendoRetracted: " + robot.intake.extendoRetracted);
        logger.info("slidesRetracted: " + robot.deposit.slidesRetracted);
        logger.info("slidesReached: " + robot.deposit.slidesReached);
        logger.info("autoEndPose: " + autoEndPose.toString());

        logger.info("hasSample(): " + robot.intake.hasSample());
        logger.info("colorSensor getDistance: " + robot.colorSensor.getDistance(DistanceUnit.CM));
        logger.info("Intake sampleColor: " + Intake.sampleColor);
        logger.info("correctSampleDetected: " + Intake.correctSampleDetected());
        logger.info("autoEndPose: " +Intake.intakeMotorState);

        logger.info( "liftTop.getPower()"+ robot.liftTop.getPower());
        logger.info("extension.getPower()"+ robot.extension.getPower());

        logger.info("getExtendoScaledPosition()"+ robot.intake.getExtendoScaledPosition());
        logger.info("getLiftScaledPosition()" + robot.deposit.getLiftScaledPosition());

        logger.info("slides target"+ robot.deposit.target);
        logger.info("extendo target"+ robot.intake.target);

        logger.info("intakePivotState"+ intakePivotState);
        logger.info("depositPivotState"+ depositPivotState);
        logger.info("Sigma"+ "Oscar");
        logger.info("botHeading"+ botHeading);
        logger.info("speedMultiplier"+ speedMultiplier);

        //logging

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
//        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());
        telemetryData.addData("extension.getPower()", robot.extension.getPower());

        telemetryData.addData("getExtendoScaledPosition()", robot.intake.getExtendoScaledPosition());
        telemetryData.addData("getLiftScaledPosition()", robot.deposit.getLiftScaledPosition());

        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("extendo target", robot.intake.target);

        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("Sigma", "Oscar");
        telemetryData.addData("botHeading", botHeading);
        telemetryData.addData("speedMultiplier", speedMultiplier);

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        deinitLogger();
        autoEndPose = robot.follower.getPose();
    }
}
