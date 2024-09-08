package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.hardware.caching.SolversAxonServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;

@Photon
@Config
public class CoaxialSwerveModule {
    private final SolversMotor motor;
    private final SolversAxonServo servo;

    // Pod rotation PIDF
    public static double P = 1.2;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    private double podTargetHeading = 0;
    private double podHeading = 0;

    private boolean motorFlipped;
    private double motorTargetPower = 0;

    private final PIDFController podPIDF;

    // Encoder offset is value of encoder when pod faces straight and when the motor runs forward the wheel also runs forward
    public CoaxialSwerveModule(SolversMotor motor, SolversAxonServo servo, double encoderOffset) {
        this.motor = motor;
        this.servo = servo;
        servo.setOffset(encoderOffset);
        podPIDF = new PIDFController(P, I, D, F);
    }

    public void init() {
        servo.setPwm(new PwmControl.PwmRange(500, 2500, 5000));
    }

    public void read() {
        podHeading = servo.getPosition();
    }

    public void update(double podTargetHeading, double motorTargetPower) {
        this.podTargetHeading = normalizeRadians(podTargetHeading);
        this.motorTargetPower = motorTargetPower;

        double error = normalizeRadians(this.podTargetHeading - this.podHeading);

        // Optimize with wheel flipping
        if (Math.abs(error) > (Math.PI / 2)) {
            this.motorFlipped = true;
            this.podTargetHeading = normalizeRadians(this.podTargetHeading + Math.PI);
        } else {
            motorFlipped = false;
        }

        error = normalizeRadians(this.podTargetHeading - this.podHeading);

        // Only for tuning purposes - remove once tuned pod PIDF or leave it :shrug:
        podPIDF.setPIDF(P, I, D, F);


        servo.setPower(podPIDF.calculate(0, error));
        motor.setPower(motorFlipped ? -this.motorTargetPower : this.motorTargetPower);

    }
}
