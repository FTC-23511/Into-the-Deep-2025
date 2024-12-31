package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum PoseLocationName {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public enum DepositInit {
        BUCKET_SCORING,
        SPECIMEN_SCORING
    }

    public static DepositInit depositInit;

    public static OpModeType opModeType;
    public static AllianceColor allianceColor;
    public static PoseLocationName poseLocationName;

    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // Intake Motor
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static double INTAKE_HOLD_SPEED = 0.15;
    public static final double SAMPLE_DISTANCE_THRESHOLD = 2.15;

    // Intake Pivot
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.25;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.835;
    public static double INTAKE_PIVOT_READY_INTAKE_POS = 0.675;

    // Intake Extendo
    public static double MAX_EXTENDO_EXTENSION = 480;
    public static double AUTO_EXTENDO_EXTENSION;

    // Deposit Pivot
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.38;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.58;
    public static double DEPOSIT_PIVOT_SCORING_POS = 1.0;
    public static double DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS = 0.765;
    public static double DEPOSIT_PIVOT_SPECIMEN_SCORING_POS = 0.00;

    // 0.84 sec/360° -> 0.828 sec/355° -> (gear ratio of 5:4) 1.035 sec/355° -> 1035 milliseconds/355°
    public static double DEPOSIT_PIVOT_MOVEMENT_TIME = 1035 + 200; // 200 milliseconds of buffer
    // 0.84 sec/360° -> 0.828 sec/355° -> 828 milliseconds/355°
    public static double INTAKE_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.65;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.4;

    // Deposit Wrist
    public static double WRIST_SCORING = 0.5;
    public static double WRIST_FRONT_SPECIMEN_SCORING = 0.5;
    public static double WRIST_BACK_SPECIMEN_SCORING = 0.5;
    public static double WRIST_FRONT_SPECIMEN_INTAKE = 0.5;
    public static double WRIST_BACK_SPECIMEN_INTAKE = 0.5;
    public static double WRIST_TRANSFER = 0.5;
    public static double WRIST_MIDDLE_HOLD = 0.5;

    // Deposit Slides
    public static double MAX_SLIDES_EXTENSION = 2050;
    public static double SLIDES_PIVOT_READY_EXTENSION = 400;
    public static double LOW_BUCKET_HEIGHT = 1000;
    public static double HIGH_BUCKET_HEIGHT = 2020;
    public static double HIGH_SPECIMEN_HEIGHT = 900;
    public static double HIGH_SPECIMEN_ATTACH_HEIGHT = 1350;
    public static double AUTO_ASCENT_HEIGHT = 900;
    public static double ENDGAME_ASCENT_HEIGHT = 1150;

    // command timeout
    public final static int MAX_COMMAND_RUN_TIME_MS = 3000;
}