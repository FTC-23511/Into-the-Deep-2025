package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.DepositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakePivotState;
import static org.firstinspires.ftc.teamcode.hardware.Globals.FRONT_HIGH_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_BUCKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.MAX_EXTENDO_EXTENSION;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.autoEndPose;
import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.HoldPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Queso (5spec+2sample)", group = "Chipotle Menu", preselectTeleOp = "SoloTeleOp")

public class Queso extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;
    public static int EXTENSION_1 = 375;
    public static int EXTENSION_2 = 343;
    public static int EXTENSION_3 = 250;
    public static int ANGLE_3 = 112;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;
    public void generatePath() {
        // If you want to edit the pathing copy and update the json code/.pp file found in the Recipes package into https://pedro-path-generator.vercel.app/
        // Then paste the following code https://pedro-path-generator.vercel.app/ spits out at you (excluding the top part with the class and constructor headers)
        // Make sure to update the Recipes package so others can update the pathing as well
        // NOTE: .setTangentialHeadingInterpolation() doesn't exist its .setTangentHeadingInterpolation() so just fix that whenever you paste

        // Starting Pose (update this as well):
        robot.follower.setStartingPose(new Pose(6.125, 66.250, Math.toRadians(0)));

        paths.add(
                // Drive to first specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(6.125, 66.250, Point.CARTESIAN),
                                        new Point(42.000, 66.250, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Drive to first sweep position for sample pushing
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(42.000, 66.250, Point.CARTESIAN),
                                        new Point(29.254, 69.619, Point.CARTESIAN),
                                        new Point(30.946, 42.925, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(145))
                        .build());

        paths.add(
                // Sweep first sample
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(30.946, 42.925, Point.CARTESIAN),
                                        new Point(28.451, 36.936, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(40))
                        .build());

        paths.add(
                // Drive to second sweep position for sample pushing
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(28.451, 36.936, Point.CARTESIAN),
                                        new Point(31.945, 31.445, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(145))
                        .build());

        paths.add(
                // Sweep second sample
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(31.945, 31.445, Point.CARTESIAN),
                                        new Point(27.452, 28.451, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(40))
                        .build());

        paths.add(
                // Drive to third sweep position for sample intaking
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(27.452, 28.451, Point.CARTESIAN),
                                        new Point(31.945, 24.458, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(ANGLE_3))
                        .build());

        paths.add(
                // Sweep third sample
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(31.945, 24.458, Point.CARTESIAN),
                                        new Point(25.955, 23.709, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(ANGLE_3), Math.toRadians(0))
                        .build());

        paths.add(
                // Move to first specimen intake minus a few inches to give human player time to align specimen
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierLine(
                                        new Point(12.000, 8.500, Point.CARTESIAN),
                                        new Point(14.000, 32.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0)).build());

        paths.add(
                // Second specimen intake (also move after minus few inches from specimen intake)
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierLine(
                                        new Point(14.000, 32.000, Point.CARTESIAN),
                                        new Point(7.000, 32.000, Point.CARTESIAN)
                                )
                        )

                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Second specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(7.500, 32.000, Point.CARTESIAN),
                                        new Point(20.000, 56.000, Point.CARTESIAN),
                                        new Point(42.000, 70.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Third specimen intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(42.000, 70.000, Point.CARTESIAN),
                                        new Point(30.000, 32.000, Point.CARTESIAN),
                                        new Point(14.000, 32.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Third specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(7.500, 32.000, Point.CARTESIAN),
                                        new Point(20.000, 56.000, Point.CARTESIAN),
                                        new Point(42.000, 68.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Fourth specimen intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 13
                                new BezierCurve(
                                        new Point(42.000, 68.500, Point.CARTESIAN),
                                        new Point(30.000, 32.000, Point.CARTESIAN),
                                        new Point(14.000, 32.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Fourth specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 14
                                new BezierCurve(
                                        new Point(7.500, 32.000, Point.CARTESIAN),
                                        new Point(20.000, 56.000, Point.CARTESIAN),
                                        new Point(42.000, 67.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Fifth specimen intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 15
                                new BezierCurve(
                                        new Point(42.000, 67.000, Point.CARTESIAN),
                                        new Point(30.000, 32.000, Point.CARTESIAN),
                                        new Point(14.000, 32.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Fifth specimen scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 16
                                new BezierCurve(
                                        new Point(7.500, 32.000, Point.CARTESIAN),
                                        new Point(20.000, 56.000, Point.CARTESIAN),
                                        new Point(42.000, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0)).build());

        paths.add(
                // Intake Sample 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 17
                                new BezierCurve(
                                        new Point(40.000, 72.000, Point.CARTESIAN),
                                        new Point(9.071, 58.961, Point.CARTESIAN),
                                        new Point(8.844, 55.332, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                        .build());

        paths.add(
                // Score Sample 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 18
                                new BezierLine(
                                        new Point(8.844, 55.332, Point.CARTESIAN),
                                        new Point(7.937, 128.579, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build());

        paths.add(
                // Intake Sample 2
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 19
                                new BezierLine(
                                        new Point(10.205, 48.529, Point.CARTESIAN),
                                        new Point(7.937, 128.579, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build());

        paths.add(
                // Score Sample 2
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 20
                                new BezierLine(
                                        new Point(10.205, 48.529, Point.CARTESIAN),
                                        new Point(7.937, 128.579, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build());
    }


    public SequentialCommandGroup intakeSpecimenCycleHalf(int pathNum) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(pathNum)),
                        new SetDeposit(robot, DepositPivotState.BACK_SPECIMEN_INTAKE, 0, true)
                ).withTimeout(4000),

                new FollowPathCommand(robot.follower, paths.get(8)).setHoldEnd(true).withTimeout(500),
                new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                new WaitCommand(200)
        );
    }

    public SequentialCommandGroup scoreSpecimenCycleHalf(int pathNum) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetDeposit(robot, DepositPivotState.FRONT_SPECIMEN_SCORING, FRONT_HIGH_SPECIMEN_HEIGHT, false).withTimeout(1000),
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true)
                ),
                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                new WaitCommand(300)
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
        FollowerConstants.zeroPowerAccelerationMultiplier = 3;

        generatePath();

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.follower.setStartingPose(new Pose(6.125, 66.250, Math.toRadians(0)))),

                        // Specimen 1
                        new ParallelCommandGroup(
                                new SetDeposit(robot, DepositPivotState.FRONT_SPECIMEN_SCORING, FRONT_HIGH_SPECIMEN_HEIGHT, false).withTimeout(1000),
                                new FollowPathCommand(robot.follower, paths.get(0)).setHoldEnd(false)
                        ),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)).andThen(
                                new WaitCommand(150)
                        ),

                        // Sample 1
                        new ParallelCommandGroup(
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true),
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE_READY, Intake.IntakeMotorState.STOP, EXTENSION_1, false),

                                new FollowPathCommand(robot.follower, paths.get(1)).setHoldEnd(true)
                        ),

                        new SetIntake(robot, Intake.IntakePivotState.HOVER, Intake.IntakeMotorState.STOP, EXTENSION_1, false).alongWith(
                                new FollowPathCommand(robot.follower, paths.get(2)).setHoldEnd(true).beforeStarting(
                                        new WaitCommand(100)
                                )
                        ),

                        new SetIntake(robot, Intake.IntakePivotState.INTAKE_READY, Intake.IntakeMotorState.STOP, EXTENSION_2, false).alongWith(
                                new FollowPathCommand(robot.follower, paths.get(3)).setHoldEnd(true)
                        ),

                        new SetIntake(robot, Intake.IntakePivotState.HOVER, Intake.IntakeMotorState.STOP, EXTENSION_2, false).alongWith(
                                new FollowPathCommand(robot.follower, paths.get(4)).setHoldEnd(true).beforeStarting(
                                        new WaitCommand(100)
                                )
                        ),

                        new SetIntake(robot, Intake.IntakePivotState.INTAKE_READY, Intake.IntakeMotorState.HOLD, EXTENSION_3, false).alongWith(
                                new FollowPathCommand(robot.follower, paths.get(5)).setHoldEnd(true)
                        ),

                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, EXTENSION_3, true),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, EXTENSION_3 + 35, true),
                        new WaitUntilCommand(() -> robot.intake.hasSample()).withTimeout(350),

                        new SetIntake(robot, Intake.IntakePivotState.HOVER, Intake.IntakeMotorState.STOP, EXTENSION_3 - 25, false).alongWith(
                                new FollowPathCommand(robot.follower, paths.get(6)).setHoldEnd(true).beforeStarting(
                                        new WaitCommand(100)
                                )
                        ),

                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.REVERSE, 0, true),
                        new WaitUntilCommand(() -> !robot.intake.hasSample()).withTimeout(350),
                        new SetIntake(robot, Intake.IntakePivotState.INSIDE, Intake.IntakeMotorState.STOP, 0, true),

                        new FollowPathCommand(robot.follower, paths.get(7)).setHoldEnd(false).alongWith(
                                new SequentialCommandGroup(
                                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.REVERSE, 0, true),
                                        new WaitUntilCommand(() -> !robot.intake.hasSample()).withTimeout(350),
                                        new SetIntake(robot, Intake.IntakePivotState.INSIDE, Intake.IntakeMotorState.STOP, 0, true)
                                )
                        ),

                        new SetDeposit(robot, DepositPivotState.BACK_SPECIMEN_INTAKE, 0, true).withTimeout(1000),

                        // Intake Specimen 2
                        new FollowPathCommand(robot.follower, paths.get(8)).setHoldEnd(true).withTimeout(500),
                        new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                        new WaitCommand(200),

                        // Score Specimen 2
                        scoreSpecimenCycleHalf(9),

                        // Intake Specimen 3
                        intakeSpecimenCycleHalf(10),

                        // Scoring Specimen 3
                        scoreSpecimenCycleHalf(11),

                        // Intake Specimen 4
                        intakeSpecimenCycleHalf(12),

                        // Scoring Specimen 4
                        scoreSpecimenCycleHalf(13),

                        // Intake Specimen 5
                        intakeSpecimenCycleHalf(14),

                        // Scoring Specimen 5
                        scoreSpecimenCycleHalf(15),

                        // Intake Sample 1
                        new ParallelCommandGroup(
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000),
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true).beforeStarting(
                                        new WaitCommand(1750)
                                ),
                                new FollowPathCommand(robot.follower, paths.get(16)).setHoldEnd(true),
                                new WaitUntilCommand(() -> robot.intake.hasSample())
                        ),

                        // Score Sample 1
                        new FollowPathCommand(robot.follower, paths.get(17)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new RealTransfer(robot),
                                        new WaitCommand(175),
                                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1000)
                                )
                        ),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                        new WaitCommand(100),
                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000)
//
//                        // Intake Sample 2
//                        new ParallelCommandGroup(
//                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true).beforeStarting(
//                                        new WaitCommand(1750)
//                                ),
//                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000),
//                                new FollowPathCommand(robot.follower, paths.get(18)).setHoldEnd(true),
//                                new WaitUntilCommand(() -> robot.intake.hasSample())
//                        ),
//
//                        // Score Sample 2
//                        new FollowPathCommand(robot.follower, paths.get(19)).setHoldEnd(true).alongWith(
//                                new SequentialCommandGroup(
//                                        new RealTransfer(robot),
//                                        new WaitCommand(175),
//                                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1000)
//                                )
//                        ),
//                        new InstantCommand(() -> robot.deposit.setClawOpen(true))
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

//        telemetry.addData("hasSample()", robot.intake.hasSample());
//        telemetry.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("followerIsBusy", robot.follower.isBusy());
        telemetry.addData("target pose", robot.follower.getClosestPose());
        telemetry.addData("Robot Pose", robot.follower.getPose());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Heading Error", Math.toDegrees(robot.follower.headingError));

        telemetry.addData("intakePivotState", intakePivotState);
        telemetry.addData("depositPivotState", depositPivotState);

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