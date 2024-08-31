package org.firstinspires.ftc.teamcode.tuning.PIDF.sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@Photon
@Config
@TeleOp
public class doubleMotorPIDF extends OpMode {
    public static int setPoint = 0;

    /*

    1. Make sure all values are 0!
    2. Move slide/arm up/down, and make sure encoder increases in positive direction.
        - If it does not, reverse either the motor direction or encoder.
     */

    public static double p = 0.00;
    public static double i = 0;
    public static double d = 0.000;
    public static double f = 0.000;
    public static double maxPowerConstant = 1.0;

    private static final PIDFController doublePIDF = new PIDFController(p,i,d, f);
    private final ExampleRobot robot = ExampleRobot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    int liftPos = robot.leftMotorEncoder.getPosition();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        doublePIDF.setTolerance(5, 10);

        robot.leftMotorEncoder.reset();

        telemetry.addData("encoder position", liftPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("max power", (f * liftPos) + maxPowerConstant);
    }

    @Override
    public void loop() {
        timer.reset();

        liftPos = robot.leftMotorEncoder.getPosition();

        doublePIDF.setP(p);
        doublePIDF.setI(i);
        doublePIDF.setD(d);
        doublePIDF.setF(f);

        doublePIDF.setSetPoint(setPoint);
        double maxPower = (f * liftPos) + maxPowerConstant;

        double power = Range.clip(doublePIDF.calculate(liftPos, setPoint), -maxPower, maxPower);
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);

        robot.ControlHub.clearBulkCache();

        telemetry.addData("encoder position", liftPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}