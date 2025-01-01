package org.firstinspires.ftc.teamcode.opmode.Auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;

@Config
@Autonomous
public class Auto0Plus1 extends OpMode {
    private final Robot robot = Robot.getInstance();
    public static int index = 0;
    public static double motorSpeeds = 0.3;
    public static int stopTimer = 500;

    public ElapsedTime timer;

    @Override
    public void init() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositInit.BUCKET_SCORING;

        CommandScheduler.getInstance().enable();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.deposit, robot.intake);

        robot.initHasMovement();
    }

    @Override
    public void loop() {
        if (timer == null) {

            timer = new ElapsedTime();

            CommandScheduler.getInstance().schedule(new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false));

            sleep(2000);

            robot.leftBack.setPower(motorSpeeds);
            robot.leftFront.setPower(motorSpeeds);
            robot.rightBack.setPower(motorSpeeds);
            robot.rightFront.setPower(motorSpeeds);

            if (index == 0) {
                index = 1;
            }
        }

        telemetry.addData("index", index);
        telemetry.addData("liftBottom power", robot.liftBottom.getPower());
        telemetry.addData("liftTop power", robot.liftTop.getPower());
        telemetry.update();

        if (timer.milliseconds() >= stopTimer && index == 1) {
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
            index = 2;
        }

        if (timer.milliseconds() >= (1000 + stopTimer) && index == 2) {
            robot.deposit.setClawOpen(true);

            sleep(500);

            robot.leftBack.setPower(-motorSpeeds);
            robot.leftFront.setPower(-motorSpeeds);
            robot.rightBack.setPower(-motorSpeeds);
            robot.rightFront.setPower(-motorSpeeds);

            timer.reset();
            index = 3;
        }

        if (timer.milliseconds() >= (1000) && index == 3) {
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);

            CommandScheduler.getInstance().schedule(new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true));

            sleep(500);

            robot.leftBack.setPower(motorSpeeds / 3);
            robot.leftFront.setPower(motorSpeeds / 3);
            robot.rightBack.setPower(motorSpeeds / 3);
            robot.rightFront.setPower(motorSpeeds / 3);

            timer.reset();
            index = 4;
        } if (timer.milliseconds() >= (1000) && index == 4) {
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
            index = 5;
        }

        CommandScheduler.getInstance().run();
    }
}