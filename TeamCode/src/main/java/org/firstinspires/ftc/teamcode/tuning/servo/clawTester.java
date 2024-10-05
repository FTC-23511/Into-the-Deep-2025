package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DriveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.System.checkButton;
import static org.firstinspires.ftc.teamcode.hardware.System.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@Photon
@Config
@TeleOp
public class clawTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final ExampleRobot robot = ExampleRobot.getInstance();
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD) {
            robot.centerServo.setPosition(CENTER_SERVO_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CENTER_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CENTER_SERVO_POS -= 0.01;
        } else if (gamepad1.dpad_right && checkButton(currentGamepad1, "dpad_right")) {
            CENTER_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            CENTER_SERVO_POS -= 0.01;
        }

        CENTER_SERVO_POS = Math.max(Math.min(CENTER_SERVO_POS, 1), 0);

        // If left joystick is down save it to that value to be used later
        if (gamepad1.square) {
            if (gamepad1.left_stick_button) {
                SQUARE_POS = CENTER_SERVO_POS;
            } else {
                robot.centerServo.setPosition(SQUARE_POS);
            }
        } else if (gamepad1.circle) {
            if (gamepad1.left_stick_button) {
                CIRCLE_POS = CENTER_SERVO_POS;
            } else {
                robot.centerServo.setPosition(CIRCLE_POS);
            }
        } else if (gamepad1.triangle) {
            if (gamepad1.left_stick_button) {
                TRIANGLE_POS = CENTER_SERVO_POS;
            } else {
                robot.centerServo.setPosition(TRIANGLE_POS);
            }
        } else if (gamepad1.x) {
            if (gamepad1.left_stick_button) {
                X_POS = CENTER_SERVO_POS;
            } else {
                robot.centerServo.setPosition(X_POS);
            }
        }

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerServo getPosition", robot.centerServo.getPosition());
        telemetry.addData("centerServoPos", round(CENTER_SERVO_POS, 2));
        telemetry.addData("trianglePos", round(TRIANGLE_POS, 2));
        telemetry.addData("squarePos", round(SQUARE_POS, 2));
        telemetry.addData("xPos", round(X_POS, 2));
        telemetry.addData("circlePos", round(CIRCLE_POS, 2));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}