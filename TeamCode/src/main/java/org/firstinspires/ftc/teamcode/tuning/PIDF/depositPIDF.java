package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class depositPIDF extends OpMode {

    // D, 0.0007
    // F, 0.00085
    // I, 0
    // maxPowerConstant, 0.5
    // P, 0.024

    public static double p = 0.00;
    public static double i = 0;
    public static double d = 0.000;
    public static double f = 0.000;

    public static int setPoint = 0;
    public static double maxPowerConstant = 1.0;

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);
    private final Robot robot = Robot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    int motorPos = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        slidePIDF.setTolerance(5, 10);

        robot.liftEncoder.reset();

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("max power", (f * motorPos) + maxPowerConstant);
    }

    @Override
    public void loop() {
        timer.reset();

        motorPos = robot.liftEncoder.getPosition();

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPos) + maxPowerConstant;
        double power = Range.clip(slidePIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

        robot.liftLeft.setPower(power);
        robot.liftRight.setPower(power);

        robot.ControlHub.clearBulkCache();

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}