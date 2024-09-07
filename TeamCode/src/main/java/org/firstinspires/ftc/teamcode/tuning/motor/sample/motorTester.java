package org.firstinspires.ftc.teamcode.tuning.motor.sample;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DriveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.System.checkButton;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CENTER_MOTOR_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

//@Photon
//@Config
//@TeleOp
public class motorTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final ExampleRobot robot = ExampleRobot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        robot.centerMotor.setPower(CENTER_MOTOR_POWER);
        robot.centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("centerMotor Power", robot.centerMotor.getPower());
        telemetry.addData("centerMotor Encoder", robot.centerMotor.getPosition());
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            robot.centerMotor.setPower(CENTER_MOTOR_POWER);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CENTER_MOTOR_POWER += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CENTER_MOTOR_POWER -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            robot.centerMotor.setPower(CENTER_MOTOR_POWER);
        }
        else{
            robot.centerMotor.setPower(0);
        }

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerMotor Power", robot.centerMotor.getPower());
        telemetry.addData("centerMotor Encoder", robot.centerMotor.getPosition());
        telemetry.update();
    }
}