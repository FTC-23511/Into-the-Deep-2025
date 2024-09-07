package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public static OpModeType opModeType;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public static DriveMode driveMode;

    public static boolean USING_IMU = true;

    public static double STARTING_X;
    public static double STARTING_Y;
    public static double STARTING_HEADING;

    // c2c distance between pods (meters)
    public static double WIDTH = 0.30235;
    public static double LENGTH = 0.34405;

    // By default values refer to servo positions, unless otherwise specified
    // By default for values that control opposite running hardware, the right value of the hardware is used
    // e.g. for ARM_TRANSFER_POS, it should correspond with the real position of the rightArm servo at the transfer

    // Deposit
    public static double ARM_TRANSFER_POS = 0.05;
    public static double ARM_BACKDROP_POS = 0.82;

    public static double WRIST_TRANSFER_POS = 0.30;

    // 4th item or 3rd with 0-index is always the default (middle horizontal)
    // Increments of 60 degrees (0.12 = 60 degrees), 0.02 IS BASELINE (flipped normal)
    public static double[] WRIST_BACKDROP_POSITIONS = {0.02, 0.20, 0.39, 0.575, 0.76, 0.945};
    public static double LEFT_CLAW_OPEN_POS = 0.41;
    public static double LEFT_CLAW_CLOSE_POS = 0.62;
    public static double RIGHT_CLAW_OPEN_POS = 0.62;
    public static double RIGHT_CLAW_CLOSE_POS = 0.41;

    // Slides
    // Encoder ticks for max extension for extendo
    public static double MAX_EXTENDO_EXTENSION = 0; // Encoder ticks
    // Encoder ticks for max extension on scoring slides
    public static double MAX_SLIDES_EXTENSION = 10000; // Encoder ticks
    // Encoder ticks for first pixel row height
    public static double FIRST_BACKDROP_ROW = 0; // Encoder ticks
    // Encoder ticks for slides between pixel row heights on backdrop
    public static double BACKDROP_INCREMENTAL_HEIGHT = 0; // Encoder ticks

    // Intake
    public static double INTAKE_POWER = 0.4; // Motor power
    public static double INTAKE_REVERSE_POWER = -0.25; // Motor power
    // Highest to lowest (lowest being normal intake flat on ground) servo positions for pitching intake
    // 0th index is fully retracted/all the way up
    public static double[] STACK_HEIGHTS = {0.5, 0.17, 0.13, 0.09, 0.05, 0.02};
    public static double TRAY_INTAKE = 0.44;
    public static double TRAY_TRANSFER = 1.0;
    // Time for distance sensor to detect a pixel until it knows it has 2 pixels in milliseconds
    public static double TWO_PIXEL_TIME = 1000;
    // Distance for distance sensor to return as lower than for a pixel to be detected in tray in centimeters
    public static double PIXEL_DISTANCE = 5;
    // After how many loops should intake distance sensor be polled
    public static int INTAKE_DISTANCE_SENSOR_POLLING = 3;
}
