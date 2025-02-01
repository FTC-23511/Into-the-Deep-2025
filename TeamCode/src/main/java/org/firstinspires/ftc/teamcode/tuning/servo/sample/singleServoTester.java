package org.firstinspires.ftc.teamcode.tuning.servo.sample;

import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.SolversMethods.checkButton;
import static org.firstinspires.ftc.teamcode.hardware.SolversMethods.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CLAW_SERVO_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

//@Photon
//@Config
//@TeleOp
public class singleServoTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final ExampleRobot robot = ExampleRobot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intakeClaw.setPosition(CLAW_SERVO_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD) {
            robot.intakeClaw.setPosition(CLAW_SERVO_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CLAW_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CLAW_SERVO_POS -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            CLAW_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            CLAW_SERVO_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross || gamepad1.triangle || gamepad1.circle) {
            robot.intakeClaw.setPosition(CLAW_SERVO_POS);
        }

        CLAW_SERVO_POS = Math.max(Math.min(CLAW_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerServo getPosition", robot.intakeClaw.getPosition());
        telemetry.addData("centerServoPos", round(CLAW_SERVO_POS, 2));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}