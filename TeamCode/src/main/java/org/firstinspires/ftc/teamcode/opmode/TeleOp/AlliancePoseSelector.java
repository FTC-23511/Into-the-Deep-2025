package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import static org.firstinspires.ftc.teamcode.hardware.Globals.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.PoseLocationName.*;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AlliancePoseSelector extends LinearOpMode {
    private ElapsedTime buttonTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Cross", "Blue Bucket (left)");
        telemetry.addData("Circle", "Blue Observation (right)");
        telemetry.addData("Square", "Red Bucket (left)");
        telemetry.addData("Triangle", "Red Observation (right)");

        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("poseLocationName", poseLocationName);

        telemetry.update();

        waitForStart();
        buttonTimer = new ElapsedTime();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.cross || gamepad2.cross) {
                allianceColor = BLUE;
                poseLocationName = BLUE_BUCKET;
            } else if (gamepad1.circle || gamepad2.circle) {
                allianceColor = BLUE;
                poseLocationName = BLUE_OBSERVATION;
            } else if (gamepad1.square || gamepad2.square) {
                allianceColor = RED;
                poseLocationName = RED_BUCKET;
            } else if (gamepad1.triangle || gamepad2.triangle) {
                allianceColor = RED;
                poseLocationName = RED_OBSERVATION;
            }

            if (gamepad1.dpad_up && buttonTimer.milliseconds() > 250) {
                subSample1.add(new Pose(1, 0, 0));
                buttonTimer.reset();
            } else if (gamepad1.dpad_down && buttonTimer.milliseconds() > 250) {
                subSample1.subtract(new Pose(1, 0, 0));
                buttonTimer.reset();
            }

            if (gamepad1.dpad_right && buttonTimer.milliseconds() > 250) {
                subSample2.add(new Pose(1, 0, 0));
                buttonTimer.reset();
            } else if (gamepad1.dpad_left && buttonTimer.milliseconds() > 250) {
                subSample2.subtract(new Pose(1, 0, 0));
                buttonTimer.reset();
            }

            telemetry.addData("Cross", "Blue Bucket (left)");
            telemetry.addData("Circle", "Blue Observation (right)");
            telemetry.addData("Square", "Red Bucket (left)");
            telemetry.addData("Triangle", "Red Observation (right)");

            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("poseLocationName", poseLocationName);

            telemetry.addData("dpad left/right", "Sub Sample Pose 1)");
            telemetry.addData("dpad up/down", "Sub Sample Pose 2)");

            telemetry.addData("Sub Sample Pose 1", subSample1);
            telemetry.addData("Sub Sample Pose 2", subSample2);

            telemetry.update();
        }
    }
}