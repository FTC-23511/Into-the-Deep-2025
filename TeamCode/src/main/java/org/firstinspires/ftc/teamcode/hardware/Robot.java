package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversAxonServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

@Photon
public class Robot {
    public SolversMotor frontLeftMotor;
    public SolversMotor frontRightMotor;
    public SolversMotor backLeftMotor;
    public SolversMotor backRightMotor;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BHI260IMU imu;
    private double rawIMUAngle = 0;
    public static double imuOffset = 0;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;


    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "frontLeftMotor"), 0.01);
        frontRightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "frontRightMotor"), 0.01);
        backLeftMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "backLeftMotor"), 0.01);
        backRightMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "backRightMotor"), 0.01);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU orientation
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Bulk reading enabled!
        // AUTO mode will bulk read by default and will redo and clear cache once the exact same read is done again
        // MANUAL mode will bulk read once per loop but needs to be manually cleared
        // Also in opModes only clear ControlHub cache as it is a hardware write
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }
        }

        imuOffset = Globals.STARTING_HEADING;

        // Inits commented out for kinematics testing
//        deposit.init();
//        intake.init();

        // Add any OpMode specific initializations here
        if (Globals.opModeType == Globals.OpModeType.AUTO) {
            // deposit.initAuto();
        } else {
            // deposit.initTeleOp();
        }
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            Thread imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        rawIMUAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    }
                }
            });
            imuThread.start();
        }
    }

    public void resetIMU() {
        imuOffset = rawIMUAngle;
    }

    public double getAngle() {
        return rawIMUAngle - imuOffset;
    }

    public static void setMecanumSpeeds(double leftX, double leftY, double rightX, double speedMultiplier) {
        if (driveMode.equals(Globals.DriveMode.FIELD_CENTRIC)) {
            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-instance.getAngle()) + leftY * Math.sin(-instance.getAngle());
            double rotY = leftX * Math.sin(-instance.getAngle()) - leftY * Math.cos(-instance.getAngle());

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);

            double frontLeftPower = ((rotY + rotX + rightX) / denominator) * speedMultiplier;
            double backLeftPower = ((rotY - rotX + rightX) / denominator) * speedMultiplier;
            double frontRightPower = ((rotY - rotX - rightX) / denominator) * speedMultiplier;
            double backRightPower = ((rotY + rotX - rightX) / denominator) * speedMultiplier;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);

        } else {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(leftY) + Math.abs((leftX * 1.1)) + Math.abs(rightX), 1);

            double frontLeftPower = (leftY + (leftX * 1.1) + rightX) / denominator;
            double backLeftPower = (leftY - (leftX * 1.1) + rightX) / denominator;
            double frontRightPower = (leftY - (leftX * 1.1) - rightX) / denominator;
            double backRightPower = (leftY + (leftX * 1.1) - rightX) / denominator;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);
        }
    }

    public static void testSetMecanumSpeeds(GamepadEx gamepad, double speedMultiplier) {
        double leftY = gamepad.getLeftY();; // Remember, Y stick value is reversed
        double leftX = gamepad.getLeftX(); // Counteract imperfect strafing
        double rightX = gamepad.getRightX();

        if (driveMode.equals(Globals.DriveMode.FIELD_CENTRIC)) {
            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-instance.getAngle()) + leftY * Math.sin(-instance.getAngle());
            double rotY = leftX * Math.sin(-instance.getAngle()) - leftY * Math.cos(-instance.getAngle());

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);

            double frontLeftPower = ((rotY + rotX + rightX) / denominator) * speedMultiplier;
            double backLeftPower = ((rotY - rotX + rightX) / denominator) * speedMultiplier;
            double frontRightPower = ((rotY - rotX - rightX) / denominator) * speedMultiplier;
            double backRightPower = ((rotY + rotX - rightX) / denominator) * speedMultiplier;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);

        } else {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(leftY) + Math.abs((leftX * 1.1)) + Math.abs(rightX), 1);

            double frontLeftPower = (leftY + (leftX * 1.1) + rightX) / denominator;
            double backLeftPower = (leftY - (leftX * 1.1) + rightX) / denominator;
            double frontRightPower = (leftY - (leftX * 1.1) - rightX) / denominator;
            double backRightPower = (leftY + (leftX * 1.1) - rightX) / denominator;

            instance.frontLeftMotor.setPower(frontLeftPower);
            instance.frontLeftMotor.setPower(backLeftPower);
            instance.frontLeftMotor.setPower(frontRightPower);
            instance.frontLeftMotor.setPower(backRightPower);
        }
    }
}
