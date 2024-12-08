package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakePivotState;
import static org.firstinspires.ftc.teamcode.hardware.Globals.AUTO_ASCENT_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.DepositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_BUCKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_SPECIMEN_ATTACH_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

import java.util.ArrayList;

@Config
@Autonomous(name = "Burrito Bowl (0spec+4sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class BurritoBowl extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;
    public void generatePath() {
        // If you want to edit the pathing copy and update the json code/.pp file found in the Recipes package into https://pedro-path-generator.vercel.app/
        // Then paste the following code https://pedro-path-generator.vercel.app/ spits out at you (excluding the top part with the class and constructor headers)
        // Make sure to update the Recipes package so others can update the pathing as well
        // NOTE: .setTangentHeadingInterpolation() doesn't exist its .setTangentHeadingInterpolation() so just fix that whenever you paste

        // Starting Pose (update this as well):
        robot.follower.setStartingPose(new Pose(6.125, 102.125, Math.toRadians(90)));

        paths.add(
                // Drive to first sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierCurve(
                                        new Point(6.125, 102.125, Point.CARTESIAN),
                                        new Point(13.726, 104.818, Point.CARTESIAN),
                                        new Point(12.977, 129.525, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build());

        paths.add(
                // Drive to second sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierLine(
                                        new Point(12.977, 129.525, Point.CARTESIAN),
                                        new Point(34.939, 122.750, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build());

        paths.add(
                // Drive to second sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(34.939, 122.750, Point.CARTESIAN),
                                        new Point(15.000, 120.00, Point.CARTESIAN),
                                        new Point(12.977, 128.776, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build());

        paths.add(
                // Drive to third sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(12.977, 128.776, Point.CARTESIAN),
                                        new Point(23.958, 130.773, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build());

        paths.add(
                // Drive to third sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(23.958, 130.773, Point.CARTESIAN),
                                        new Point(14.974, 130.024, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build());

        paths.add(
                // Drive to fourth sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(14.974, 130.024, Point.CARTESIAN),
                                        new Point(23.709, 132.520, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-158)).build());

        paths.add(
                // Drive to fourth sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(23.709, 132.520, Point.CARTESIAN),
                                        new Point(13.974, 130.776, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-158), Math.toRadians(135)).build());

        paths.add(
                // Park/ascent level 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(13.974, 130.776, Point.CARTESIAN),
                                        new Point(63.706, 117.899, Point.CARTESIAN),
                                        new Point(62.157, 93.500 , Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90)).build());
    }
    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositInit.BUCKET_SCORING;

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        robot.follower.setMaxPower(0.6);

        generatePath();

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        // Sample 1
                        new ParallelCommandGroup(
                                new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new FollowPathCommand(robot.follower, paths.get(0))
                                )
                        ),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Sample 2
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(1)),
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 0, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                )
                        ),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new WaitCommand(2000)
                        ),
                        new RealTransfer(robot),
                        new WaitCommand(250),
                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                        new FollowPathCommand(robot.follower, paths.get(2)),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Sample 3
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(3)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                )
                        ),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 0, true),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 220, true),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new WaitCommand(2000)
                        ),
                        new RealTransfer(robot),
                        new WaitCommand(250),
                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                        new FollowPathCommand(robot.follower, paths.get(4)),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Sample 4
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(5)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                )
                        ),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 0, true),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 300 , true),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new WaitCommand(2000)
                        ),
                        new RealTransfer(robot),
                        new WaitCommand(250),
                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                        new FollowPathCommand(robot.follower, paths.get(6)),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Park
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(7)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true),
                                        new InstantCommand(() -> robot.follower.setMaxPower(0.4))
                                )
                        ),

                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, AUTO_ASCENT_HEIGHT, true)
                )
        );


        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
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
        telemetry.addData("liftEncoder.getPosition()", robot.extensionEncoder.getPosition());

        telemetry.update(); // DO NOT REMOVE! Needed for telemetry

        // Pathing telemetry
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}