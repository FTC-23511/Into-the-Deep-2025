package org.firstinspires.ftc.teamcode.tuning.PIDF.sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

//@Photon
//@Config
//@TeleOp
public class singleMotorPIDF extends OpMode {

    // D, 0.000_
    // F, 0.000_
    // I, 0
    // maxPowerConstant, 1.0
    // P, 0.00_

    public static double p = 0.00;
    public static double i = 0;
    public static double d = 0.000;
    public static double f = 0.000;

    public static int setPoint = 0;
    public static double maxPowerConstant = 1.0;

    private static final PIDFController singlePIDF = new PIDFController(p,i,d, f);
    private final ExampleRobot robot = ExampleRobot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    int motorPos = robot.centerMotor.getPosition();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        singlePIDF.setTolerance(5, 10);

        robot.centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("max power", (f * motorPos) + maxPowerConstant);
    }

    @Override
    public void loop() {
        timer.reset();

        motorPos = robot.centerMotor.getPosition();

        singlePIDF.setP(p);
        singlePIDF.setI(i);
        singlePIDF.setD(d);
        singlePIDF.setF(f);

        singlePIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPos) + maxPowerConstant;
        double power = Range.clip(singlePIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

        robot.centerMotor.setPower(power);

        robot.ControlHub.clearBulkCache();

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}