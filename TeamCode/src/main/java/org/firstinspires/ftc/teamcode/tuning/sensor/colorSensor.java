package org.firstinspires.ftc.teamcode.tuning.sensor;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Photon
@TeleOp(name = "colorSensor", group = "Sensor")
@Config
public class colorSensor extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        RevColorSensorV3 colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("color");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()) {
            timer.reset();

            colorSensor.enableLed(true);

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            double distance = colorSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("RGB", "(" + red + ", " + green + ", " + blue + ")");

            telemetry.addData("distance (CM)", distance);

            String color = "";

            if (distance < 3.5) {
                if (red >= green && red >= blue) {
                    color = "red";
                } else if (green >= red && green >= blue) {
                    color = "green";
                } else {
                    color = "blue"; // Set default to be blue if on blue side, set default to be red if on red side.
                }
            }

            telemetry.addData("color", color);

            /* Misc Color Sensor Tools:
            int alpha = colorSensor.alpha(); //0 - Transparent, 255 - Opaque
            int combined = colorSensor.argb();
            telemetry.addData("alpha", alpha);
            telemetry.addData("combined", combined);
            */

            telemetry.addData("loop time (ms)", timer.milliseconds());
            telemetry.update();
        }
    }
}
