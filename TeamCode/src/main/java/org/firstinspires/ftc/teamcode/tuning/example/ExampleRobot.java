package org.firstinspires.ftc.teamcode.tuning.example;


import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotorEncoder;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;

import java.util.List;

@Photon
public class ExampleRobot {
    public SolversServo leftServo;
    public SolversServo rightServo;
    public SolversServo centerServo;
    public SolversMotor centerMotor;
    public SolversMotor leftMotor;
    public SolversMotor rightMotor;
    public SolversMotorEncoder centerMotorEncoder;
    public SolversMotorEncoder leftMotorEncoder;
    public SolversMotorEncoder rightMotorEncoder;

    private static ExampleRobot instance = null;
    public boolean enabled;

    public static ExampleRobot getInstance() {
        if (instance == null) {
            instance = new ExampleRobot();
        }
        instance.enabled = true;
        return instance;
    }

    public List<LynxModule> allHubs;
    public LynxModule ControlHub;

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) { // CONFIG: robotTester
        leftServo = new SolversServo("leftServo", 0.0); // Servo Slot 4 on Control Hub
        rightServo = new SolversServo("rightServo", 0.0); // Servo Slot 5 on Control Hub
        centerServo = new SolversServo("centerServo", 0.0); // Servo Slot 0 on Control Hub

//        leftServo.setDirection(Servo.Direction.REVERSE);

        centerMotor = new SolversMotor("centerMotor", 0.01); // Motor Slot 0 on Control Hub
        centerMotorEncoder = new SolversMotorEncoder(centerMotor);

        leftMotor = new SolversMotor("leftMotor", 0.01); // Motor Slot 2 on Control Hub
        leftMotorEncoder = new SolversMotorEncoder(leftMotor);

        rightMotor = new SolversMotor("rightMotor", 0.01); // Motor Slot 3 on Control Hub
        rightMotorEncoder = new SolversMotorEncoder(rightMotor);

        centerMotorEncoder.setDirection(SolversMotorEncoder.Direction.REVERSE);
        centerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

    }
}
