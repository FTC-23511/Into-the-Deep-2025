package org.firstinspires.ftc.teamcode.tuning.example;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import com.seattlesolvers.solverslib.solversHardware.SolversMotor;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;

import java.util.List;

public class IntakeSubsystem {
    public SolversServo leftIntakePivot;
    public SolversServo wrist;
    public SolversServo rightIntakePivot;
    public SolversMotor centerMotor;
    public SolversServo intakeClaw;
    public RevColorSensorV3 colorSensor;
    public Deposit deposit;
    public Intake intake;
    public Motor.Encoder liftEncoder;

    private static IntakeSubsystem instance = null;
    public boolean enabled;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        instance.enabled = true;
        return instance;
    }

    public List<LynxModule> allHubs;
    public LynxModule ControlHub;

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) { // CONFIG: robotTester
        intakeClaw = new SolversServo(hardwareMap.get(Servo.class, "intakeClaw"), 0.0); // Servo Slot 0 on Control Hub
        leftIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "leftServo"), 0.0); // Servo Slot 1 on Control Hub
        rightIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "rightServo"), 0.0); // Servo Slot 2 on Control Hub
        wrist = new SolversServo(hardwareMap.get(Servo.class, "wrist"), 0.0); // Servo Slot 2 on Control Hub
        colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");
        leftIntakePivot.setDirection(Servo.Direction.REVERSE);
//
//        centerMotor = new SolversDcMotor(hardwareMap.get(DcMotor.class, "centerMotor"), 0.01); // Motor Slot 0 on Control Hub
//        liftEncoder = new Motor(hardwareMap, "centerMotor").encoder;
//        leftMotor = new SolversDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftMotor"), 0.01); // Motor Slot 1 on Control Hub
//        rightMotor = new SolversDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightMotor"), 0.01); // Motor Slot 2 on Control Hub
//        centerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        liftBottom = new SolversDcMotorEx((hardwareMap.get(DcMotorEx.class, "liftBottom")), 0.01);
//        liftTop = new SolversDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftTop"), 0.01);
//        encoder = new MotorEx(hardwareMap, "liftTop").encoder;

//        liftTop.setDirection(DcMotorEx.Direction.REVERSE);
//        encoder.setDirection(MotorEx.Direction.REVERSE);

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
    }
}
