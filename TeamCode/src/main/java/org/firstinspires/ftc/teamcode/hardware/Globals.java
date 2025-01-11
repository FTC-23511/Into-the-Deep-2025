package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;

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

    public static Deposit.DepositPivotState depositInit;

    public static OpModeType opModeType;
    public static AllianceColor allianceColor;
    public static PoseLocationName poseLocationName;

    public static Pose subSample1;
    public static Pose subSample2;


    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // Intake Motor
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static double INTAKE_HOLD_SPEED = 0.15;

    // Intake Color Sensor
    public static final double MIN_DISTANCE_THRESHOLD = 1;
    public static final double MAX_DISTANCE_THRESHOLD = 1.6;
    public static final int YELLOW_THRESHOLD = 0;
    public static final int RED_THRESHOLD = 0;
    public static final int BLUE_THRESHOLD = 0;

    // Intake Pivot
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.05;
    public static double INTAKE_PIVOT_READY_TRANSFER_POS = 0.19;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.735;
    public static double INTAKE_PIVOT_READY_INTAKE_POS = 0.54;
    public static double INTAKE_PIVOT_HOVER_INTAKE_POS = 0.71;

    // Intake Extendo
    public static double MAX_EXTENDO_EXTENSION = 500;

    // Deposit Pivot
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.9442;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.98;
    public static double DEPOSIT_PIVOT_AUTO_BAR_POS = 0.35;
    public static double DEPOSIT_PIVOT_SCORING_POS = 0.3;
    public static double DEPOSIT_PIVOT_SPECIMEN_FRONT_INTAKE_POS = 0.07;
    public static double DEPOSIT_PIVOT_SPECIMEN_BACK_INTAKE_POS = 0.83;
    public static double DEPOSIT_PIVOT_SPECIMEN_FRONT_SCORING_POS = 0.20;
    public static double DEPOSIT_PIVOT_SPECIMEN_BACK_SCORING_POS = 0.71;

    // 0.84 sec/360° -> 0.828 sec/355° -> (gear ratio of 5:4) 1.035 sec/355° -> 1035 milliseconds/355°
    public static double DEPOSIT_PIVOT_MOVEMENT_TIME = 1035 + 200; // 200 milliseconds of buffer
    // 0.84 sec/360° -> 0.828 sec/355° -> 828 milliseconds/355°
    public static double INTAKE_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.48;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.18;

    // Deposit Wrist
    public static double WRIST_SCORING = 0.37;
    public static double WRIST_AUTO_BAR = 0.3;
    public static double WRIST_FRONT_SPECIMEN_SCORING = 0.60;
    public static double WRIST_BACK_SPECIMEN_SCORING = 0.43;
    public static double WRIST_FRONT_SPECIMEN_INTAKE = 0.33;
    public static double WRIST_BACK_SPECIMEN_INTAKE = 0.5;
    public static double WRIST_TRANSFER = 0.4;
    public static double WRIST_MIDDLE_HOLD = 0.4;

    // Deposit Slides
    public static double MAX_SLIDES_EXTENSION = 2000;
    public static double SLIDES_PIVOT_READY_EXTENSION = 450;
    public static double LOW_BUCKET_HEIGHT = 450;
    public static double HIGH_BUCKET_HEIGHT = 1900;
    public static double FRONT_HIGH_SPECIMEN_HEIGHT = 1050;
    public static double BACK_HIGH_SPECIMEN_HEIGHT = 800;
    public static double BACK_HIGH_SPECIMEN_ATTACH_HEIGHT = 1400;
    public static double AUTO_ASCENT_HEIGHT = 800;
    public static double ENDGAME_ASCENT_HEIGHT = 1150;

    // Hang Servos
    public static double LEFT_HANG_FULL_POWER = 0.9;
    public static double RIGHT_HANG_FULL_POWER = 1.0;

    // Gearbox Switcher Servo
    public static double HANG_GEAR_POS = 0.0;
    public static double DEPOSIT_GEAR_POS = 0.0;

    // Sub Pusher / Sweeper Servo
    public static double SUB_PUSHER_OUT = 0.54;
    public static double SUB_PUSHER_IN = 0.08;
    public static double SUB_PUSHER_AUTO = 0.5;

    // command timeout
    public final static int MAX_COMMAND_RUN_TIME_MS = 3000;
}