package org.firstinspires.ftc.teamcode.hardware.caching;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

/**
 * A wrapper servo class that provides caching to avoid unnecessary setPower() calls.
 * Credit to team FTC 22105 (Runtime Terror) for the base class, we just modified it
 */

public class SolversCRServo {
    private double lastPower = 0;
    private final PhotonCRServo crservo;

    private double powerThreshold = 0.0;

    public SolversCRServo(PhotonCRServo crservo, double powerThreshold) {
        this.crservo = crservo;
        this.powerThreshold = powerThreshold;
    }

    public void setPower(double power) {
        if ((Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            crservo.setPower(power);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public void setPwm(PwmControl.PwmRange pwmRange) {
        ServoControllerEx servoController = (ServoControllerEx) crservo.getController();
        servoController.setServoPwmRange(crservo.getPortNumber(), pwmRange);

    }

    public double getPower() {
        return lastPower;
    }

}