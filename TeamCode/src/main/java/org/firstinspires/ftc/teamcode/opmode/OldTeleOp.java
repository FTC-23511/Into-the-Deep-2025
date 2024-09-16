package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Robot.setMecanumSpeeds;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositBackdrop;
import org.firstinspires.ftc.teamcode.subsystem.commands.transfer;

@Photon
@TeleOp
public class OldTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer = null;
    public ElapsedTime buttonTimer = null;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake, robot.swerveDrivetrain);
        robot.deposit.initTeleOp();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        // Only run first time, but not in init because IMU thread would start polling?
        // TO-DO: Need to check if this is needed or if it is safe to put in initialize()
        if (timer == null) {
            timer = new ElapsedTime();
            buttonTimer = new ElapsedTime();
            // DO NOT REMOVE! Gets the IMU readings on separate thread
            robot.startIMUThread(this);

        }
        // Endgame/hang rumble after 110 seconds to notify driver to hang
        else if ((timer.seconds() > 110) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        // Runs the command scheduler and performs all the periodic() functions of each subsystem
        super.run();

        // Driving stuff
        // Minimum power of 0.2 and scale trigger value by remainder
        // Value to scale power to drivetrain based on driver trigger
        double speedMultiplier = (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.8) + 0.2;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driver.getLeftX() * STRAFE_MULTIPLIER * speedMultiplier,
                driver.getLeftY() * STRAFE_MULTIPLIER * speedMultiplier,
                driver.getRightX() * TURN_MULTIPLIER * speedMultiplier,
                new Rotation2d(robot.getAngle())
        );

        robot.swerveDrivetrain.update(chassisSpeeds);

        // For Mecanum:
        setMecanumSpeeds(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), speedMultiplier);
//        testSetMecanumSpeeds(driver, speedMultiplier);

        // Driver buttons
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.intake.setExtendoTarget(MAX_EXTENDO_EXTENSION);
        } else if ((driver.wasJustPressed(GamepadKeys.Button.B))) {
            robot.intake.setExtendoTarget(0);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            new transfer(robot.deposit, robot.intake);
        }

        if (driver.isDown(GamepadKeys.Button.Y)) {
            robot.intake.setIntake(Intake.IntakeState.REVERSED_ON);
        } else if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
            robot.intake.setIntake(Intake.IntakeState.ON);
        } else {
            robot.intake.setIntake(Intake.IntakeState.OFF);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.intake.stackHeight < 5) {
            robot.intake.stackHeight += 1;
        } else if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)  && robot.intake.stackHeight > 0) {
            robot.intake.stackHeight -= 1;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            robot.imu.resetYaw();
        }

        if (driver.isDown(GamepadKeys.Button.DPAD_UP)) {
            robot.intake.setExtendoTarget(robot.extensionEncoder.getPosition() + 10); // Needs to be tested
        } else if (driver.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            robot.intake.setExtendoTarget(robot.extensionEncoder.getPosition() - 10); // Needs to be tested
        }

        robot.intake.setPitchingIntake(robot.intake.stackHeight);

        // Operator buttons
        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            new depositBackdrop(robot.deposit);
        } else if (operator.wasJustPressed(GamepadKeys.Button.B)) {
            robot.deposit.setSlideTarget(0);
        }

        if ((operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) && buttonTimer.milliseconds() >= 200) {
            robot.deposit.wristIndex -= 1;
            robot.deposit.moveWrist();
            buttonTimer.reset();
        } else if ((operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) && buttonTimer.milliseconds() >= 200) {
            robot.deposit.wristIndex += 1;
            robot.deposit.moveWrist();
            buttonTimer.reset();
        }

        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.deposit.teleOpSetClaw(!robot.deposit.leftClawOpen, robot.deposit.leftClawOpen);
        } else if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.deposit.teleOpSetClaw(!robot.deposit.leftClawOpen, robot.deposit.leftClawOpen);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            robot.deposit.teleOpSetClaw(true, true  );
        } else if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            robot.deposit.teleOpSetClaw(false, false);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP) && robot.deposit.pixelHeight < 10) {
            robot.deposit.pixelHeight += 1;
        }

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}