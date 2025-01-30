package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.RescheduleScoring;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

import java.util.ArrayList;

@Config
@Autonomous(name = "Lifestyle Bowl (0spec+6sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class LifestyleBowl extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    private final ArrayList<PathChain> paths = new ArrayList<>();
    TelemetryData telemetryData = new TelemetryData(telemetry);
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
                                        new Point(14.000, 126.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130)).build());

        paths.add(
                // Drive to second sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierLine(
                                        new Point(14.000, 126.000, Point.CARTESIAN),
                                        new Point(22.960, 127.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(161)).build());

        paths.add(
                // Drive to second sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(22.960, 127.000, Point.CARTESIAN),
                                        new Point(15.500, 133.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(161)).build());

        paths.add(
                // Drive to third sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(15.500, 130.000, Point.CARTESIAN),
                                        new Point(23.958, 130.100, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(161), Math.toRadians(180)).build());

        paths.add(
                // Drive to third sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(23.958, 130.100, Point.CARTESIAN),
                                        new Point(15.500, 133.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(161)).build());

        paths.add(
                // Drive to fourth sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(15.500, 133.000, Point.CARTESIAN),
                                        new Point(23.709, 132.520, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(161), Math.toRadians(-157)).build());

        paths.add(
                // Drive to fourth sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(23.709, 132.520, Point.CARTESIAN),
                                        new Point(15.500, 133.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-157), Math.toRadians(161)).build());

        paths.add(
                // Drive from bucket to (subSample1)
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(13.974, 130.776, Point.CARTESIAN),
                                        new Point(subSample1.getX(), 118.000, Point.CARTESIAN),
                                        new Point(subSample1)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(161), Math.toRadians(90)).build());

        paths.add(
                // Drive from submersible to bucket (subSample1)
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(subSample1),
                                        new Point(subSample1.getX(), 118.000, Point.CARTESIAN),
                                        new Point(13.974, 130.776, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build());

        paths.add(
                // Drive from bucket to (subSample2)
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(13.974, 130.776, Point.CARTESIAN),
                                        new Point(subSample2.getX(), 118.000, Point.CARTESIAN),
                                        new Point(subSample2)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90)).build());

        paths.add(
                // Drive from submersible to bucket (subSample2)
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(subSample2),
                                        new Point(subSample2.getX(), 118.000, Point.CARTESIAN),
                                        new Point(13.974, 130.776, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build());

        paths.add(
                // Park/ascent level 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(14.500, 130.000, Point.CARTESIAN),
                                        new Point(62.000, 118.000, Point.CARTESIAN),
                                        new Point(62.000, 95.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90)).build());
    }

    private SequentialCommandGroup intakeSampleCycleHalf(int pathNum, int extendoTarget) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 120, true),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000)
                        )
                ),
                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, extendoTarget, true).withTimeout(1000),

                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.intake::hasSample)
//                        ,
//                        new SequentialCommandGroup(
//                                new FollowPathCommand(robot.follower, robot.jiggle(5)),
//                                new FollowPathCommand(robot.follower, robot.jiggle(5))
//                        )
                ).withTimeout(1000),

                // Allow sample to enter intake fully
                new WaitCommand(100),
                new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.HOLD)),
                // Allow tubing to hold onto sample
                new WaitCommand(100),
                new RealTransfer(robot)
        );
    }

    private SequentialCommandGroup scoreSampleCycleHalf(int pathNum) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true)
                        ),
                        new RescheduleScoring(
                                robot,
                                new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1000),
                                robot.deposit.depositFailed
                        ).withTimeout(7000)
                ),
                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                new WaitCommand(200)
        );
    }

    private SequentialCommandGroup intakeSubSampleCycleHalf(int pathNum) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000)
                        )
                ),

                new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.OUT)),
                new WaitCommand(300),
                new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.IN)),
                new WaitCommand(300),

                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 15, true).withTimeout(500),
                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true).withTimeout(2000),

                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> Intake.correctSampleDetected() && robot.intake.hasSample())
//                        ,
//                        new SequentialCommandGroup(
//                                new FollowPathCommand(robot.follower, robot.jiggle(5)),
//                                new FollowPathCommand(robot.follower, robot.jiggle(5))
//                        )
                ).withTimeout(1000),

                // Allow sample to enter intake fully
                new WaitCommand(100),
                new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.HOLD)),
                // Allow tubing to hold onto sample
                new WaitCommand(100)
        );
    }

    private SequentialCommandGroup scoreSubSampleCycleHalf(int pathNum) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> FollowerConstants.zeroPowerAccelerationMultiplier = 1.5),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RealTransfer(robot),
                                new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1000)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true)
                        )
                ),
                new InstantCommand(() -> FollowerConstants.zeroPowerAccelerationMultiplier = 2),
                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                new WaitCommand(200)
        );
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

        robot.follower.setMaxPower(1);
        FollowerConstants.zeroPowerAccelerationMultiplier = 1.5;

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
                                        new FollowPathCommand(robot.follower, paths.get(0)).setHoldEnd(true)
                                )
                        ),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                        new WaitCommand(250),

                        // Sample 2
                        intakeSampleCycleHalf(1, 320),
                        scoreSampleCycleHalf(2),

                        // Sample 3
                        intakeSampleCycleHalf(3, 320),
                        scoreSampleCycleHalf(4),

                        // Sample 4
                        intakeSampleCycleHalf(5, 350),
                        scoreSampleCycleHalf(6),

                        // Sub Sample 1
                        intakeSubSampleCycleHalf(7),
                        scoreSubSampleCycleHalf(8),

                        // Sub Sample 2
                        intakeSubSampleCycleHalf(9),
                        scoreSubSampleCycleHalf(10),

                        // Park
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(11)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, DepositPivotState.AUTO_TOUCH_BAR, 0, false)
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

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("extendoReached", robot.intake.extendoReached);
        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("robotState", Robot.robotState);

        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("depositFailed", robot.deposit.depositFailed);
        telemetryData.addData("redoDeposit()", robot.deposit.redoDeposit(depositPivotState));
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));

        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);

        telemetryData.addData("UndoTransfer", CommandScheduler.getInstance().isScheduled(new UndoTransfer(robot)));
        telemetryData.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());

        telemetryData.addData("deposit target", robot.deposit.target);
        telemetryData.addData("liftEncoder.getPosition()", robot.liftEncoder.getPosition());
        telemetryData.addData("extendo target", robot.intake.target);
        telemetryData.addData("extensionEncoder.getPosition()", robot.extensionEncoder.getPosition());

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry

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