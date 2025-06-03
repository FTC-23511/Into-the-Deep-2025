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

    public static boolean soloTeleOp = false;

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
    public static AllianceColor allianceColor = AllianceColor.BLUE;
    public static PoseLocationName poseLocationName;

    public static Pose subSample1 = new Pose(62.000, 93.700, Math.toRadians(90));
    public static Pose subSample2 = new Pose(62.000, 93.700, Math.toRadians(90));
    public static Pose autoEndPose = new Pose(0, 0, Math.toRadians(0));


    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // Intake Motor
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -0.5;
    public static double INTAKE_HOLD_SPEED = 0.15;
    public static int REVERSE_TIME_MS = 300;

    // Intake Color Sensor
    public static double MIN_DISTANCE_THRESHOLD = 1.0;
    public static double MAX_DISTANCE_THRESHOLD = 1.5;
    public static int YELLOW_THRESHOLD = 800;
    public static int RED_THRESHOLD = 0;
    public static int BLUE_THRESHOLD = 0;

    public static int YELLOW_EDGE_CASE_THRESHOLD = 1450;
    public static int RED_EDGE_CASE_THRESHOLD = 700;
    public static int BLUE_EDGE_CASE_THRESHOLD = 675;

    // Intake Pivot
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.22;
    public static double INTAKE_PIVOT_INSIDE_POS = 0.1;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.735;
    public static double INTAKE_PIVOT_READY_INTAKE_POS = 0.54;
    public static double INTAKE_PIVOT_HOVER_INTAKE_POS = 0.72;

    // Intake Extendo
    public static double MAX_EXTENDO_EXTENSION = 525; // Previously 500

    // Deposit Pivot
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.81;
    public static double DEPOSIT_PIVOT_READY_TRANSFER_POS = 0.90;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.7;
    public static double DEPOSIT_PIVOT_INSIDE_POS = 1.00;
    public static double DEPOSIT_PIVOT_SCORING_POS = 0.35;
    public static double DEPOSIT_PIVOT_SPECIMEN_FRONT_INTAKE_POS = 0.05;
    public static double DEPOSIT_PIVOT_SPECIMEN_BACK_INTAKE_POS = 0.83;
    public static double DEPOSIT_PIVOT_SPECIMEN_FRONT_SCORING_POS = 0.20;
    public static double DEPOSIT_PIVOT_SPECIMEN_BACK_SCORING_POS = 0.71;

    // 0.84 sec/360° -> 0.828 sec/355° -> 828 milliseconds/355°
    public static double DEPOSIT_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer
    // 0.84 sec/360° -> 0.828 sec/355° -> (gear ratio of 48:80) 0.497 sec/355° -> 497 milliseconds/355°
    public static double INTAKE_PIVOT_MOVEMENT_TIME = 497 + 200; // 200 milliseconds of buffer

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.43;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.14;

    // Deposit Wrist
    public static double WRIST_SCORING = 0.45;
    public static double WRIST_INSIDE = 0.4;
    public static double WRIST_FRONT_SPECIMEN_SCORING = 0.60;
    public static double WRIST_BACK_SPECIMEN_SCORING = 0.43;
    public static double WRIST_FRONT_SPECIMEN_INTAKE = 0.3;
    public static double WRIST_BACK_SPECIMEN_INTAKE = 0.485;
    public static double WRIST_TRANSFER = 0.19;
    public static double WRIST_MIDDLE_HOLD = 0.20;
    public static double WRIST_READY_TRANSFER = 0.22;

    // Deposit Slides
    public static double MAX_SLIDES_EXTENSION = 2000;
    public static double SLIDES_PIVOT_READY_EXTENSION = 450;
    public static double LOW_BUCKET_HEIGHT = 450;
    public static double HIGH_BUCKET_HEIGHT = 2000;
    public static double FRONT_HIGH_SPECIMEN_HEIGHT = 1065;
    public static double BACK_HIGH_SPECIMEN_HEIGHT = 850;
    public static double BACK_HIGH_SPECIMEN_ATTACH_HEIGHT = 1450;
    public static double AUTO_ASCENT_HEIGHT = 800;
    public static double ENDGAME_ASCENT_HEIGHT = 1300;

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