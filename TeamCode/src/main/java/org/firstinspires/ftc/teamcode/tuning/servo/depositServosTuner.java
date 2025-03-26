package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.SolversMethods.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class depositServosTuner extends OpMode {
    private final Robot robot = Robot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.leftDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.rightDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.depositWrist.setPosition(WRIST_SERVO_POS);
        robot.depositClaw.setPosition(CLAW_SERVO_POS);

        robot.leftIntakePivot.setPosition(INTAKE_SERVO_POS);
        robot.rightIntakePivot.setPosition(INTAKE_SERVO_POS);
    }

    @Override
    public void loop() {
        robot.leftDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.rightDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.depositWrist.setPosition(WRIST_SERVO_POS);
        robot.depositClaw.setPosition(CLAW_SERVO_POS);
        robot.leftIntakePivot.setPosition(INTAKE_SERVO_POS);
        robot.rightIntakePivot.setPosition(INTAKE_SERVO_POS);

        CENTER_SERVO_POS = Math.max(Math.min(CENTER_SERVO_POS, 1), 0);
        WRIST_SERVO_POS = Math.max(Math.min(WRIST_SERVO_POS, 1), 0);
        CLAW_SERVO_POS = Math.max(Math.min(CLAW_SERVO_POS, 1), 0);
        INTAKE_SERVO_POS = Math.max(Math.min(INTAKE_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        if (gamepad1.a) {
            robot.deposit.setClawOpen(true);
        }
        if (gamepad1.b) {
            robot.deposit.setClawOpen(false);
        }

        telemetry.addData("leftIntakePivot getPosition", robot.leftIntakePivot.getPosition());
        telemetry.addData("rightIntakePivot getPosition",robot.rightIntakePivot.getPosition());
        telemetry.addData("leftDepositPivot getPosition", robot.leftDepositPivot.getPosition());
        telemetry.addData("rightDepositPivot getPosition",robot.rightDepositPivot.getPosition());
        telemetry.addData("depositWrist getPosition",robot.depositWrist.getPosition());
        telemetry.addData("depositClaw getPosition",robot.depositClaw.getPosition());
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}