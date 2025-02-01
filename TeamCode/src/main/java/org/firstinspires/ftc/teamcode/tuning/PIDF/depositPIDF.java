package org.firstinspires.ftc.teamcode.tuning.PIDF;

import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class depositPIDF extends OpMode {
    public static double p = 0.005; // Old: 0.011
    public static double i = 0;
    public static double d = 0.00017; // Old: 0.0002
    public static double f = 0.00023; // Old: 0.00016

    public static int setPoint = 0;
    public static double maxPowerConstant = 1.0;
    private final static int divideConstant = 30;

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);
    private final Robot robot = Robot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    int motorPos = 0;

    @Override
    public void init() {
        opModeType = Globals.OpModeType.TELEOP;

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

        motorPos = robot.liftEncoder.getPosition() / divideConstant;

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPos) + maxPowerConstant;
        double power = Range.clip(slidePIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

        robot.liftBottom.setPower(power);
        robot.liftTop.setPower(power);

        robot.ControlHub.clearBulkCache();

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}