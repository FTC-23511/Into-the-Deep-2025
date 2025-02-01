package org.firstinspires.ftc.teamcode.tuning.motor;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@Config
@TeleOp
public class motorEncoders extends OpMode {
    public static boolean RESET_ALL_ENCODERS = false;
    public static boolean RESET_LIFT_ENCODER = false;
    public static boolean RESET_EXTENSION_ENCODER = false;
    public static boolean RESET_PARALLEL_ENCODER = false;
    public static boolean RESET_PERPENDICULAR_ENCODER = false;

    private final Robot robot = Robot.getInstance();
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;

        // Resets command scheduler
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);

        telemetry.addData("liftEncoder", robot.liftEncoder.getPosition());
        telemetry.addData("extensionEncoder", robot.extensionEncoder.getPosition());
    }

    @Override
    public void loop() {
        timer.reset();

        if (RESET_ALL_ENCODERS) {
            robot.liftEncoder.reset();
            robot.extensionEncoder.reset();
        }

        robot.ControlHub.clearBulkCache();

        telemetry.addData("liftEncoder", robot.liftEncoder.getPosition());
        telemetry.addData("extensionEncoder", robot.extensionEncoder.getPosition());

        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}