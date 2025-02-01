package org.firstinspires.ftc.teamcode.tuning.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class depositFullTuner extends OpMode {
    private final Robot robot = Robot.getInstance();

    public static double p = 0.005; // Old: 0.011
    public static double i = 0;
    public static double d = 0.00017; // Old: 0.0002
    public static double f = 0.00023; // Old: 0.00016

    public static int setPoint = 0;
    public static double maxPowerConstant = 1.0;
    public static int divideConstant = 30;

    public ElapsedTime timer = new ElapsedTime();

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);
    int motorPos = 0;
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        slidePIDF.setTolerance(5, 10);

        robot.liftEncoder.reset();

        robot.leftDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.rightDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.depositWrist.setPosition(WRIST_SERVO_POS);
        robot.depositClaw.setPosition(CLAW_SERVO_POS);

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

        robot.leftDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.rightDepositPivot.setPosition(CENTER_SERVO_POS);
        robot.depositWrist.setPosition(WRIST_SERVO_POS);
        robot.depositClaw.setPosition(CLAW_SERVO_POS);

        CENTER_SERVO_POS = Math.max(Math.min(CENTER_SERVO_POS, 1), 0);
        WRIST_SERVO_POS = Math.max(Math.min(WRIST_SERVO_POS, 1), 0);
        CLAW_SERVO_POS = Math.max(Math.min(CLAW_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        if (gamepad1.a) {
            robot.deposit.setClawOpen(true);
        }
        if (gamepad1.b) {
            robot.deposit.setClawOpen(false);
        }

        telemetry.addData("leftDepositPivot getPosition", robot.leftDepositPivot.getPosition());
        telemetry.addData("rightDepositPivot getPosition",robot.rightDepositPivot.getPosition());
        telemetry.addData("depositWrist getPosition",robot.depositWrist.getPosition());
        telemetry.addData("depositClaw getPosition",robot.depositClaw.getPosition());

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}