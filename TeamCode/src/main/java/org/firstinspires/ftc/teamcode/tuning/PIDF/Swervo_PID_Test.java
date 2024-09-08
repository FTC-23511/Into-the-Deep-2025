package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class Swervo_PID_Test extends OpMode {
    public static double setPoint = 0;

    public static boolean fL = false;
    public static boolean fR = false;
    public static boolean bL = false;
    public static boolean bR = false;

    public static double fLoffset = -2.524;
    public static double fRoffset = 0.529;
    public static double bLoffset = -2.163;
    public static double bRoffset = -1.832;

    public static double motorPower = 0;

    private final Robot robot = Robot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        timer.reset();

        if (fL) {
            robot.frontLeftServo.setOffset(fLoffset);
            robot.fL.read();
            robot.fL.update(setPoint, motorPower);
            telemetry.addData("fL encoder position", robot.frontLeftServo.getPosition());
            telemetry.addData("fL servoPower", robot.frontLeftServo.getPower());
        }
        if (fR) {
            robot.frontRightServo.setOffset(fRoffset);
            robot.fR.read();
            robot.fR.update(setPoint, motorPower);
            telemetry.addData("fR encoder position", robot.frontRightServo.getPosition());
            telemetry.addData("fR servoPower", robot.frontRightServo.getPower());
        }
        if (bL) {
            robot.backLeftServo.setOffset(bLoffset);
            robot.bL.read();
            robot.bL.update(setPoint, motorPower);
            telemetry.addData("bL encoder position", robot.backLeftServo.getPosition());
            telemetry.addData("bL servoPower", robot.backLeftServo.getPower());
        }
        if (bR) {
            robot.backRightServo.setOffset(bRoffset);
            robot.bR.read();
            robot.bR.update(setPoint, motorPower);
            telemetry.addData("bR encoder position", robot.backRightServo.getPosition());
            telemetry.addData("bR servoPower", robot.backRightServo.getPower());
        }

        telemetry.addData("setPoint", setPoint);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();

        robot.ControlHub.clearBulkCache();
    }
}