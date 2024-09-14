package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.System.checkButton;
import static org.firstinspires.ftc.teamcode.hardware.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class trayTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final Robot robot = Robot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.tray.setPosition(TRAY_TRANSFER);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD) {
            robot.tray.setPosition(TRAY_TRANSFER);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            TRAY_TRANSFER += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            TRAY_TRANSFER -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            TRAY_TRANSFER += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            TRAY_TRANSFER -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross || gamepad1.triangle || gamepad1.circle) {
            robot.tray.setPosition(TRAY_TRANSFER);
        }

        TRAY_TRANSFER = Math.max(Math.min(TRAY_TRANSFER, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("tray getPosition", robot.tray.getPosition());
        telemetry.addData("trayPos", round(TRAY_TRANSFER, 2));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}