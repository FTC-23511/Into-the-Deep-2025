package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.seattlesolvers.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous
public class LimeLight extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private final ArrayList<Position> reads = new ArrayList<>();
    IMU imu;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = Globals.OpModeType.AUTO;
        depositInit = Globals.DepositInit.SPECIMEN_SCORING;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        telemetry.setMsTransmissionInterval(11);
        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        imu.resetYaw();

        timer.reset();
    }

    @Override
    public void run() {
        super.run();
        robot.follower.update();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

        robot.limelight.updateRobotOrientation(Math.toDegrees(robot.follower.getPose().getHeading()));
        LLResult result = robot.limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT2 = result.getBotpose_MT2();

//                reads.add(result.getBotpose_MT2().getPosition());

                reads.add(new Position(DistanceUnit.INCH,
                        result.getBotpose().getPosition().x,
                        result.getBotpose().getPosition().y,
                        result.getBotpose().getPosition().z,
                        result.getBotpose().getPosition().acquisitionTime));


                telemetry.addData("MT2 Pose:", "(" + result.getBotpose().getPosition().x + ", " + result.getBotpose().getPosition().y + ", " + robot.follower.getPose().getHeading() + ")");
                telemetry.update();
            }
        }

        if (gamepad1.a) {
            imu.resetYaw();
            robot.follower.setPose(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), 0));
        }

        if (gamepad1.b) {
            if (!reads.isEmpty()) {
                // Averaging
                double x = 0;
                double y = 0;

                for (Position read : reads) {
                    x += read.x;
                    y += read.y;
                }

                x /= reads.size();
                y /= reads.size();

                // Switch to pedro 0" to 144" coordinate system instead of -72" to 72" coordinate system
                x += 72;
                y += 72;

                robot.follower.setPose(new Pose(x , y, robot.follower.getPose().getHeading()));
//            previousReads = reads;
            }
        }

        telemetry.addData("Robot Pose:", "(" + robot.follower.getPose().getX() + ", " + robot.follower.getPose().getY() + ", " + robot.follower.getPose().getHeading() + ")");

        telemetry.addData("Reads Size:", Objects.toString(reads.size()));

        telemetry.addData("OTOS Heading (deg):", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("OTOS Heading read():", robot.follower.getPose().getHeading());

        telemetry.addData("Reads:", Objects.toString(reads));

        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}