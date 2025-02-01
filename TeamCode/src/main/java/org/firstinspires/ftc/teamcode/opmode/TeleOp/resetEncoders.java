package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "AAAresetEncoders")
public class resetEncoders extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    private FtcDashboard dash = FtcDashboard.getInstance();

    public ElapsedTime timer;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        robot.liftTop.setPower(gamepad1.left_stick_x);
        robot.liftBottom.setPower(gamepad1.left_stick_x);

        robot.extension.setPower(gamepad1.right_stick_x);

        robot.liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("liftEncoder", robot.liftEncoder.getPosition());
        telemetry.addData("extensionEncoder", robot.extensionEncoder.getPosition());

        // DO NOT REMOVE! Needed for telemetry
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}