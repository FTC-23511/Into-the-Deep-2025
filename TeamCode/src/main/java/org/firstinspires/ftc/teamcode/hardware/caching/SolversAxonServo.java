package org.firstinspires.ftc.teamcode.hardware.caching;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

/**
 * A wrapper CRServo class for axon servos that has two goals.
 * 1) Provide caching to avoid unnecessary setPower() lynxcommands.
 * 2) Allow for easy usage of the Axon servos.
 * Credit to 22105 Runtime Terror for majority of the class
 */

@Photon
public class SolversAxonServo {
    private double offset = 0;
    private double lastPower = 0;
    private AnalogInput servoEncoder = null;
    private final PhotonCRServo crservo;

    private double powerThreshold = 0;

    public SolversAxonServo(@NonNull PhotonCRServo crservo, double powerThreshold) {
        this.crservo = crservo;
        this.powerThreshold = powerThreshold;
        this.lastPower = crservo.getPower();
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    /**
     * Sets the servo encoder.
     * @param absoluteEncoder The analog input of the absolute encoder.
     */
    public void setServoEncoder(AnalogInput absoluteEncoder) {
        this.servoEncoder = absoluteEncoder;
    }

    synchronized public void setPower(double power) {
        if (Math.abs(this.lastPower - power) > this.powerThreshold) {
            this.lastPower = power;
            this.crservo.setPower(Range.clip(power, -1, 1));
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

    /**
     * Returns the CURRENT position of the axon servo.
     * @return The current, actual position of the servo in radians. This is ABSOLUTE.
     */
    public double getRawPosition() {
        return this.servoEncoder.getVoltage() / 3.3 * Math.PI*2;
    }

    public void setPwm(PwmControl.PwmRange pwmRange) {
        ServoControllerEx servoController = (ServoControllerEx) crservo.getController();
        servoController.setServoPwmRange(crservo.getPortNumber(), pwmRange);
    }

    /**
     * Returns the normalized position of the axon servo.
     * @return The normalized position.
     */
    public double getPosition() {
        return normalizeRadians(getRawPosition() - offset);
    }

    /**
     * Sets an offset to be subtracted to the return value of getPosition()
     * @param offset The offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getPower() {
        return lastPower;
    }
}