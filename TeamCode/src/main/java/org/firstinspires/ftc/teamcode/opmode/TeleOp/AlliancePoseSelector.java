package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import static org.firstinspires.ftc.teamcode.hardware.Globals.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.PoseLocation.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AlliancePoseSelector extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Cross", "Blue Bucket (left)");
        telemetry.addData("Circle", "Blue Observation (right)");
        telemetry.addData("Square", "Red Bucket (left)");
        telemetry.addData("Triangle", "Red Observation (right)");

        telemetry.addData("Current Pose", startingPose);

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.cross) {
                startingPoseName = BLUE_BUCKET;
                allianceColor = BlUE;
            } else if (gamepad1.circle) {
                startingPoseName = BLUE_OBSERVATION;
                allianceColor = BlUE;
            } else if (gamepad1.square) {
                startingPoseName = RED_BUCKET;
                allianceColor = RED;
            } else if (gamepad1.triangle) {
                startingPoseName = RED_OBSERVATION;
                allianceColor = RED;
            }

            startingPose = STARTING_POSES.get(startingPoseName);

            telemetry.addData("Cross", "Blue Bucket (left)");
            telemetry.addData("Circle", "Blue Observation (right)");
            telemetry.addData("Square", "Red Bucket (left)");
            telemetry.addData("Triangle", "Red Observation (right)");

            telemetry.addData("Current Pose", startingPoseName);
            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("startingPose", startingPose);

            telemetry.update();
        }
    }
}