package org.firstinspires.ftc.teamcode.tuning.servo.sample;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DriveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.System.checkButton;
import static org.firstinspires.ftc.teamcode.hardware.System.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.LEFT_SERVO_POS;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.RIGHT_SERVO_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

//@Photon
//@Config
//@TeleOp
public class doubleServoTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    public static boolean MOVE_ONE = true;
    public static boolean MOVE_BOTH = false;
    private final ExampleRobot robot = ExampleRobot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.leftServo.setPosition(LEFT_SERVO_POS);
        robot.rightServo.setPosition(RIGHT_SERVO_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD && MOVE_ONE) {
            robot.leftServo.setPosition(LEFT_SERVO_POS);
            robot.rightServo.setPosition(RIGHT_SERVO_POS);
        } else if (USE_DASHBOARD && MOVE_BOTH) {
            robot.leftServo.setPosition(LEFT_SERVO_POS);
            robot.rightServo.setPosition(RIGHT_SERVO_POS);
            MOVE_ONE = false;
            MOVE_BOTH = false;
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            LEFT_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            LEFT_SERVO_POS -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            RIGHT_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            RIGHT_SERVO_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross) {
            robot.leftServo.setPosition(LEFT_SERVO_POS);
        } else if (gamepad1.triangle || gamepad1.circle) {
            robot.rightServo.setPosition(RIGHT_SERVO_POS);
        }

        LEFT_SERVO_POS = Math.max(Math.min(LEFT_SERVO_POS, 1), 0);
        RIGHT_SERVO_POS = Math.max(Math.min(RIGHT_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("leftServo getPosition", robot.leftServo.getPosition());
        telemetry.addData("rightServo getPosition",robot.rightServo.getPosition());
        telemetry.addData("leftServoPos", round(LEFT_SERVO_POS, 2));
        telemetry.addData("rightServoPos", round(RIGHT_SERVO_POS, 2));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}