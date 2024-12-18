package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import static org.firstinspires.ftc.teamcode.hardware.Globals.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.PoseLocationName.*;

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

        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("poseLocationName", poseLocationName);

        telemetry.update();

        waitForStart();
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

            telemetry.addData("Cross", "Blue Bucket (left)");
            telemetry.addData("Circle", "Blue Observation (right)");
            telemetry.addData("Square", "Red Bucket (left)");
            telemetry.addData("Triangle", "Red Observation (right)");

            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("poseLocationName", poseLocationName);

            telemetry.update();
        }
    }
}