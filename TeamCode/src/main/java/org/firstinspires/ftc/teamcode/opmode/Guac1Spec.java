package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.DepositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakePivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakePivotState;
import static org.firstinspires.ftc.teamcode.hardware.Globals.BACK_HIGH_SPECIMEN_ATTACH_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.FRONT_HIGH_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Guacamole (1spec+0sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class Guac1Spec extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;
    public void generatePath() {
        // If you want to edit the pathing copy and update the json code/.pp file found in the Recipes package into https://pedro-path-generator.vercel.app/
        // Then paste the following code https://pedro-path-generator.vercel.app/ spits out at you (excluding the top part with the class and constructor headers)
        // Make sure to update the Recipes package so others can update the pathing as well
        // NOTE: .setTangentialHeadingInterpolation() doesn't exist its .setTangentHeadingInterpolation() so just fix that whenever you paste

        // Starting Pose (update this as well):
        robot.follower.setStartingPose(new Pose(6.125, 66.25, Math.toRadians(0)));

        paths.add(
                // Drive to first specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(6.125, 66.250, Point.CARTESIAN),
                                        new Point(40.250, 66.250, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());
        // sussy park
        paths.add(
                // Move to first specimen intake minus a few inches to give human player time to align specimen
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6 (8 in Guacamole pathing)
                                new BezierLine(
                                        new Point(40.250, 66.250, Point.CARTESIAN),
                                        new Point(12.000, 26.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build());

        paths.add(
                // Drive to first sample spike mark
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(40.250, 66.250, Point.CARTESIAN),
                                        new Point(31.695, 52.908, Point.CARTESIAN),
                                        new Point(32.500, 39.432, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130))
                        .build());

        paths.add(
                // Drop off first sample into observation zone
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(32.500, 39.432, Point.CARTESIAN),
                                        new Point(29.449, 33.692, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(30)).build());

        paths.add(
                // Drive to second sample spike mark
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(29.449, 33.692, Point.CARTESIAN),
                                        new Point(33.000, 31.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(130)).build());

        paths.add(
                // Drop off second sample into observation zone
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(33.000, 31.000, Point.CARTESIAN),
                                        new Point(28.950, 23.709, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(30)).build());

        paths.add(
                // Move to first specimen intake minus a few inches to give human player time to align specimen
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6 (8 in Guacamole pathing)
                                new BezierLine(
                                        new Point(16.000, 5.750, Point.CARTESIAN),
                                        new Point(12.000, 26.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(180)).build());

        paths.add(
                // Second specimen intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7 (9 in Guacamole pathing) (modified heading so it matches above heading)
                                new BezierLine(
                                        new Point(12.000, 26.000, Point.CARTESIAN),
                                        new Point(6.250, 26.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Second specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8 (10 in Guacamole pathing)
                                new BezierCurve(
                                        new Point(6.250, 26.000, Point.CARTESIAN),
                                        new Point(14.000, 64.000, Point.CARTESIAN),
                                        new Point(42.500, 64.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Third specimen intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9 (11 in Guacamole pathing)
                                new BezierCurve(
                                        new Point(42.500, 64.000, Point.CARTESIAN),
                                        new Point(30.000, 26.000, Point.CARTESIAN),
                                        new Point(6.250, 26.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Third specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10 (12 in Guacamole pathing)
                                new BezierCurve(
                                        new Point(6.250, 26.000, Point.CARTESIAN),
                                        new Point(14.000, 66.000, Point.CARTESIAN),
                                        new Point(42.500, 66.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Fourth specimen intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11 (13 in Guacamole pathing)
                                new BezierCurve(
                                        new Point(42.500, 66.000, Point.CARTESIAN),
                                        new Point(30.000, 26.000, Point.CARTESIAN),
                                        new Point(6.250, 26.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Fourth specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                        // Line 12 (14 in Guacamole)
                        new BezierCurve(
                                new Point(6.250, 26.000, Point.CARTESIAN),
                                new Point(14.000, 70.000, Point.CARTESIAN),
                                new Point(42.500, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Park in observation zone
                robot.follower.pathBuilder()
                        .addPath(
                        // Line 13 (15 in Guacamole)
                        new BezierCurve(
                                new Point(42.500, 70.000, Point.CARTESIAN),
                                new Point(30.000, 26.000, Point.CARTESIAN),
                                new Point(18.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180)).build());
    }

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositPivotState.FRONT_SPECIMEN_SCORING;

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        robot.follower.setMaxPower(0.45);

        generatePath();


        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        // Specimen 1
                        new ParallelCommandGroup(
                                new SetDeposit(robot, DepositPivotState.FRONT_SPECIMEN_SCORING, FRONT_HIGH_SPECIMEN_HEIGHT, false),
                                new FollowPathCommand(robot.follower, paths.get(0))
                        ),

                        new WaitCommand(1000),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                        new WaitCommand(3000),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(1)),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, false)
                                )
                        )
                )
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.poseUpdater);
        Drawing.drawRobot(robot.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("extendoReached", robot.intake.extendoReached);
        telemetry.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetry.addData("slidesReached", robot.deposit.slidesReached);
        telemetry.addData("robotState", Robot.robotState);

        telemetry.addData("intakePivotState", intakePivotState);
        telemetry.addData("depositPivotState", depositPivotState);

        telemetry.addData("UndoTransfer", CommandScheduler.getInstance().isScheduled(new UndoTransfer(robot)));
        telemetry.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetry.addData("liftBottom.getPower()", robot.liftBottom.getPower());

        telemetry.addData("deposit target", robot.deposit.target);
        telemetry.addData("liftEncoder.getPosition()", robot.liftEncoder.getPosition());
        telemetry.addData("extendo target", robot.intake.target);
        telemetry.addData("extensionEncoder.getPosition()", robot.extensionEncoder.getPosition());

        telemetry.addData("Robot Pose", robot.follower.getPose());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));

        telemetry.addData("Heading Error", Math.toDegrees(robot.follower.headingError));

        telemetry.update(); // DO NOT REMOVE! Needed for telemetry

        // Pathing telemetry
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}