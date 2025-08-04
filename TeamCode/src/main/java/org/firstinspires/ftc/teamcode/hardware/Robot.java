package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import com.seattlesolvers.solverslib.solversHardware.SolversMotor;
import com.seattlesolvers.solverslib.solversHardware.SolversMotorEx;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;
import com.seattlesolvers.solverslib.solversHardware.SolversCRServo;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import com.pedropathing.follower.FollowerConstants;

import java.util.List;

public class Robot {
    public SolversMotorEx liftBottom;
    public SolversMotorEx liftTop;
    public SolversMotorEx extension;
    public SolversMotorEx intakeMotor;

    public SolversMotorEx leftFront;
    public SolversMotorEx rightFront;
    public SolversMotorEx leftBack;
    public SolversMotorEx rightBack;

    public SolversServo leftIntakePivot;
    public SolversServo rightIntakePivot;

    public SolversServo leftDepositPivot;
    public SolversServo rightDepositPivot;
    public SolversServo depositClaw;
    public SolversServo depositWrist;

    public SolversServo subPusher;

    public SolversCRServo leftHang;
    public SolversCRServo rightHang;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder extensionEncoder;

    public RevColorSensorV3 colorSensor;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;

//    public Limelight3A limelight;

    public Deposit deposit;
    public Intake intake;
    public Drive drive;

    public Follower follower;
    public PoseUpdater poseUpdater;

    public IMU imu;

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
        liftBottom = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "liftBottom"), 0.01);
        liftTop = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "liftTop"), 0.01);
        extension = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "extension"), 0.01);
        intakeMotor = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"), 0.01);

        leftFront = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "FL"), 0.01);
        rightFront = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "FR"), 0.01);
        leftBack = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "BL"), 0.01);
        rightBack = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "BR"), 0.01);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftEncoder = new Motor(hardwareMap, "liftTop").encoder;
        extensionEncoder = new Motor(hardwareMap, "extension").encoder;
        liftEncoder.setDirection(Motor.Direction.REVERSE);
        extensionEncoder.setDirection(Motor.Direction.REVERSE);

        leftIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "leftIntakePivot"), 0.01);
        rightIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "rightIntakePivot"), 0.01);
        leftDepositPivot = new SolversServo(hardwareMap.get(Servo.class, "leftDepositPivot"), 0.01);
        rightDepositPivot = new SolversServo(hardwareMap.get(Servo.class, "rightDepositPivot"), 0.01);
        depositClaw = new SolversServo(hardwareMap.get(Servo.class, "depositClaw"), 0.01);
        depositWrist = new SolversServo(hardwareMap.get(Servo.class, "depositWrist"), 0.01);
        leftHang = new SolversCRServo(hardwareMap.get(CRServo.class, "leftHang"), 0.01);
        rightHang = new SolversCRServo(hardwareMap.get(CRServo.class, "rightHang"), 0.01);
<<<<<<< Updated upstream
        subPusher = new SolversServo(hardwareMap.get(Servo.class, "subPusher"), 0.01);
=======
>>>>>>> Stashed changes

        leftIntakePivot.setDirection(Servo.Direction.REVERSE);
        leftDepositPivot.setDirection(Servo.Direction.REVERSE);
        rightHang.setDirection(CRServo.Direction.REVERSE);
        extension.setDirection(CRServo.Direction.REVERSE);

        colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");

        colorSensor.enableLed(true);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        initializeImu(hardwareMap);

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

        intake = new Intake();
        deposit = new Deposit();
        drive = new Drive();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        FollowerConstants.useBrakeModeInTeleOp = false;

        poseUpdater = new PoseUpdater(hardwareMap);

        if (opModeType.equals(OpModeType.TELEOP)) {
            follower.startTeleopDrive();
            INTAKE_HOLD_SPEED = 0;

            follower.setStartingPose(autoEndPose);
        } else {
            follower.setStartingPose(new Pose(0, 0, 0));
//            limelight.pipelineSwitch(1);
//            limelight.start();
            INTAKE_HOLD_SPEED = 0.15;
        }
    }

    public void initHasMovement() {
        deposit.init();
        intake.init();
        drive.init();

        robotState = RobotState.MIDDLE_RESTING;
    }

    public double getMotorVoltages() {
        return intakeMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public double getPitchDegrees() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }
    public double getRollDegrees() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void initializeImu(HardwareMap hardwareMap) {
        // IMU orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public enum RobotState {
        MIDDLE_RESTING,
        TRANSFERRED,
        SCORING,
        INTAKING,
        EJECTING,
        SPECIMEN_INTAKING,
        SPECIMEN_SCORING
    }

    public static RobotState robotState;
}