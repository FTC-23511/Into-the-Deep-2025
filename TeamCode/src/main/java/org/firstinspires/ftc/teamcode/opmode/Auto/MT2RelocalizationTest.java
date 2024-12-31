package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commandbase.commands.MT2Relocalization;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Objects;

@Config
//@TeleOp
@Autonomous
public class MT2RelocalizationTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    double x = 0;
    double y = 0;
    double heading = 0;
    private final ArrayList<Position> reads = new ArrayList<>();
    private Position latestRead = new Position();

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.AUTO;
        depositInit = DepositInit.SPECIMEN_SCORING;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void run() {
        super.run();

        schedule(new MT2Relocalization(robot, 3));

        robot.follower.update();

        heading =
                // Limelight needs heading in degrees
                Math.toDegrees(
                        // Limelight needs -180 to 180 normalized angle (here its -PI to PI)
                        AngleUnit.normalizeRadians(
                                // Actual heading
                                robot.follower.getPose().getHeading()
                                        // Compensate for limelight heading being off by 90 (0.5 pi) degrees compared to Pedro
                                        - Math.PI/2
                        )
                );

        robot.limelight.updateRobotOrientation(heading);

        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            //  reads.add(result.getBotpose().getPosition());
//                reads.add(new Position(DistanceUnit.INCH,
//                        result.getBotpose().getPosition().x,
//                        result.getBotpose().getPosition().y,
//                        result.getBotpose().getPosition().z,
//                        result.getBotpose().getPosition().acquisitionTime));

            latestRead = result.getBotpose_MT2().getPosition();
        }

//        if (!reads.isEmpty()) {
//            // Averaging
//            for (Position read : reads) {
//                x += read.x;
//                y += read.y;
//            }
//
//            x /= reads.size();
//            y /= reads.size();
//
//            // Switch to pedro 0" to 144" coordinate system instead of -72" to 72" coordinate system
//            x += 72;
//            y += 72;
//
//            robot.follower.setPose(new Pose(x , y, robot.follower.getPose().getHeading()));
//        }

        telemetry.addData("MT2 Pose:", "(" + latestRead.x + ", " + latestRead.y + ", " + heading + ")");
        telemetry.addData("MT2 Pose (in Pedro)", "(" + (-latestRead.y * 39.3701 + 72) + ", " + (latestRead.x * 39.3701 + 72) + ", " + robot.follower.getPose().getHeading() + ")");
        telemetry.addData("Robot Follower Pose:", "(" + robot.follower.getPose().getX() + ", " + robot.follower.getPose().getY() + ", " + robot.follower.getPose().getHeading() + ")");

//        telemetry.addData("Reads:", Objects.toString(reads));
//        telemetry.addData("Reads Size:", Objects.toString(reads.size()));

        telemetry.addData("OTOS Heading (deg):", Math.toDegrees(robot.follower.getPose().getHeading()));

        telemetry.update();
    }
}