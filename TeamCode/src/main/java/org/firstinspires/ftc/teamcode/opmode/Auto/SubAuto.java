package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Drawing;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;

import java.util.ArrayList;

@Config
@Autonomous(name = "Sub Auto", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class SubAuto extends CommandOpMode {
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
        robot.follower.setStartingPose(new Pose(6.125, 78, Math.toRadians(90)));

        paths.add(

                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                        new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY() - 3, Point.CARTESIAN)
                                )
                        )
                        .setTangentHeadingInterpolation().build());

        paths.add(

                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(6.125, 75.500, Point.CARTESIAN),
                                        new Point(6.125, 78.000, Point.CARTESIAN)
                                )
                        )
                        .setTangentHeadingInterpolation().build());

    }
    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositPivotState.SCORING;

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        robot.intake.setPivot(IntakePivotState.INTAKE_READY);

        robot.follower.setMaxPower(0.45);

        generatePath();

//        ParallelRaceGroup subIntake = new ParallelRaceGroup(
//                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, false),
//                new WaitCommand(2000),
//                new WaitUntilCommand(robot.intake::hasSample)
//        );

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        // Need to get to sub clear position and sub clear first tho

                        new InstantCommand(() -> robot.intake.setPivot(IntakePivotState.INTAKE)),

                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new SetIntake(robot, IntakePivotState.INTAKE, FORWARD, MAX_EXTENDO_EXTENSION, false),
                                                new WaitCommand(1000)
                                        ),
                                        new FollowPathCommand(robot.follower, paths.get(0)),
                                        new FollowPathCommand(robot.follower, robot.jiggle(2.0)),
                                        new WaitCommand(500),
                                        new FollowPathCommand(robot.follower, robot.jiggle(-2.0))
                                ),
                                new WaitCommand(4000),
                                new WaitUntilCommand(robot.intake::hasSample)
                        ),

                        new ConditionalCommand(
                                new InstantCommand(),
                                new WaitCommand(500),
                                () -> Intake.correctSampleDetected() || !robot.intake.hasSample()
                        ),

                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new SequentialCommandGroup(
                                        new SetIntake(robot, IntakePivotState.INTAKE_READY, FORWARD, 50, true),

                                        new FollowPathCommand(robot.follower, paths.get(0)),
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, robot.jiggle(2.5)),

                                                new ParallelRaceGroup(
                                                        new SetIntake(robot, IntakePivotState.INTAKE, FORWARD, MAX_EXTENDO_EXTENSION, false),
                                                        new WaitCommand(2000),
                                                        new WaitUntilCommand(robot.intake::hasSample)
                                                )
                                        ),

                                        new FollowPathCommand(robot.follower, robot.jiggle(-5)),

                                        new SetIntake(robot, IntakePivotState.INTAKE_READY, FORWARD, 50, true),

                                        new ParallelRaceGroup(
                                                new SetIntake(robot, IntakePivotState.INTAKE, FORWARD, MAX_EXTENDO_EXTENSION, false),
                                                new WaitCommand(2000),
                                                new WaitUntilCommand(robot.intake::hasSample)
                                        ),

                                        new FollowPathCommand(robot.follower, robot.jiggle(5))
                                    )
                        ),

                        new RealTransfer(robot)
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

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}