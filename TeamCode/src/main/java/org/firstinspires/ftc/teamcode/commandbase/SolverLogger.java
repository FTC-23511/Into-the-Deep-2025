package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakePivotState;
import static org.firstinspires.ftc.teamcode.hardware.Globals.autoEndPose;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.EventTeleOp;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.EventTeleOp2;


import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;


public class SolverLogger {

    String logFileName;
    private FileHandler logFileHandler;

    public ElapsedTime timer;

    private Logger logger;

    public GamepadEx driver;


    private final Robot robot = Robot.getInstance();


    public SolverLogger(String fileName)  {
        logFileName = fileName;
        timer = new ElapsedTime();

    }

    public boolean init() {
        if (logger != null) {
            return true;
        }

        try {
            // Create a FileHandler to write logs to a file named "my_application.log"
            // The 'true' argument means to append to the file if it exists,
            // otherwise, a new file will be created.
            String fullLogPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + logFileName;
            logFileHandler = new FileHandler(fullLogPath, false);

            //Simple Formatter
            SimpleFormatter formatter = new SimpleFormatter();
            logFileHandler.setFormatter(formatter);

            logger = Logger.getLogger(EventTeleOp2.class.getName());
            logger.addHandler(logFileHandler);
            logger.setLevel(Level.INFO);

            logger = Logger.getLogger(EventTeleOp.class.getName());
            // Add the FileHandler to the logger.
            logger.addHandler(logFileHandler);

            logger.setLevel(Level.INFO);

        } catch (SecurityException | IOException e) {
            e.printStackTrace();
            return false;
        }

        return true;
    }

    public void deinit() {
        logger.info("Deinitialize the logger");
        logger = null;
        logFileHandler.close();
        logFileHandler = null;
    }

    public void log(double speedMultiplier) {
        String str = "";
        double botHeading = robot.getYawDegrees();


        // log format
        // [Name:Type:Value];
        // E.g.
        // Timestamp:int64:value;Pose:Tuple:{a,b,c};

        str += "timer:Double:" + timer.milliseconds();
        str += ";autoEndPose:Pose:" + autoEndPose.toString();
        str += ";extendoReached:Boolean:" + robot.intake.extendoReached;
        str += ";extendoRetracted:Boolean:" + robot.intake.extendoRetracted;
        str += ";slidesRetracted:Boolean:" + robot.deposit.slidesRetracted;
        str += ";slidesReached:Boolean:" + robot.deposit.slidesReached;
        str += ";autoEndPose:Pose:" + autoEndPose.toString();

        str += ";hasSample():Boolean:" + robot.intake.hasSample();
        str += ";colorSensor getDistance:Double:" + robot.colorSensor.getDistance(DistanceUnit.CM);
        str += ";Intake sampleColor:String:" + Intake.sampleColor;
        str += ";correctSampleDetected:Boolean:" + Intake.correctSampleDetected();
        str += ";intakeMotorState:String:" +Intake.intakeMotorState;

        str +=  ";liftTop.getPower():Double:"+ robot.liftTop.getPower();
        str += ";extension.getPower():Double:"+ robot.extension.getPower();

        str += ";getExtendoScaledPosition():Double:"+ robot.intake.getExtendoScaledPosition();
        str += ";getLiftScaledPosition():Double:" + robot.deposit.getLiftScaledPosition();

        str += ";slides target:Double:"+ robot.deposit.target;
        str += ";extendo target:Double:"+ robot.intake.target;

        str += ";intakePivotState:String:"+ intakePivotState;
        str += ";depositPivotState;String:"+ depositPivotState;
        str += ";botHeading:Double:"+ botHeading;
        str += ";speedMultiplier:Double:"+ speedMultiplier;

        logger.info(str);
    }


}
