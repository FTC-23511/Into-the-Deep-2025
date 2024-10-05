package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;

import java.nio.channels.Pipe;

@TeleOp
@Config
public class LM0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        SolversMotor frontLeftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "FL"));
        SolversMotor backLeftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "BL"));
        SolversMotor frontRightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "FR"));
        SolversMotor backRightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "BR"));

        SolversMotor armMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "armMotor"));
        SolversServo claw = new SolversServo(hardwareMap.get(PhotonServo.class, "claw"), 0.01);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm stuff
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFController armPIDF = new PIDFController(0, 0, 0, 0);
        armPIDF.setTolerance(10, 10);
        armPIDF.setSetPoint(armMotor.getPosition());
        final int ARM_INTAKE_POS = 0;
        final int ARM_CHAMBER_POS = 0;
        final int ARM_BASKET_POS = 0;
        final int ARM_HANGING_POS = 0;

        // Claw stuff
        final double CLAW_OPEN_POS = 0.5;
        final double CLAW_CLOSE_POS = 0.5;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double speedMultiplier = 0.35 + (1 - 0.35) * gamepad1.left_trigger;

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            if (gamepad1.triangle) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * speedMultiplier;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * speedMultiplier;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * speedMultiplier;
            double backRightPower = ((rotY + rotX - rx) / denominator) * speedMultiplier;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Arm and claw stuff
            if (gamepad2.left_stick_button) {
                armMotor.resetEncoder();
            } else if (gamepad2.a) {
                armPIDF.setSetPoint(ARM_INTAKE_POS);
                claw.setPosition(CLAW_OPEN_POS);
            } else if (gamepad2.b) {
                armPIDF.setSetPoint(ARM_CHAMBER_POS);
                claw.setPosition(CLAW_CLOSE_POS);
            } else if (gamepad2.x) {
                armPIDF.setSetPoint(ARM_BASKET_POS);
                claw.setPosition(CLAW_CLOSE_POS);
            } else if (gamepad2.y) {
                armPIDF.setSetPoint(ARM_HANGING_POS);
            }

            if (gamepad2.left_bumper) {
                claw.setPosition(CLAW_OPEN_POS);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(CLAW_CLOSE_POS);
            }

            if (gamepad2.left_trigger > 0.01) {
                armMotor.setPower(-Math.sqrt(gamepad2.left_trigger));
            } else if (gamepad2.right_trigger > 0.01) {
                armMotor.setPower(Math.sqrt(gamepad2.right_trigger));
            } else if (gamepad2.dpad_up) {
                armPIDF.setSetPoint(armPIDF.getSetPoint() + 10);
            } else if (gamepad2.dpad_down) {
                armPIDF.setSetPoint(armPIDF.getSetPoint() - 10);
            } else {
                armMotor.setPower(armPIDF.calculate(armMotor.getPosition()));
            }
        }
    }
}