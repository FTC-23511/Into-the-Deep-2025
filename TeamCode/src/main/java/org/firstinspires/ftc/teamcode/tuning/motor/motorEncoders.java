package org.firstinspires.ftc.teamcode.tuning.motor;

import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
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
        opModeType = Globals.OpModeType.TELEOP;
        driveMode = Globals.DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);

        telemetry.addData("liftEncoder", robot.liftEncoder.getPosition());
        telemetry.addData("extensionEncoder", robot.extensionEncoder.getPosition());
        telemetry.addData("parallelPod", robot.parallelPod.getPosition());
        telemetry.addData("perpendicularPod", robot.perpendicularPod.getPosition());
    }

    @Override
    public void loop() {
        timer.reset();

        if (RESET_ALL_ENCODERS) {
            robot.liftEncoder.reset();
            robot.extensionEncoder.reset();
            robot.parallelPod.reset();
            robot.perpendicularPod.reset();
        }

        robot.ControlHub.clearBulkCache();

        telemetry.addData("liftEncoder", robot.liftEncoder.getPosition());
        telemetry.addData("extensionEncoder", robot.extensionEncoder.getPosition());
        telemetry.addData("parallelPod", robot.parallelPod.getPosition());
        telemetry.addData("perpendicularPod", robot.perpendicularPod.getPosition());
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}