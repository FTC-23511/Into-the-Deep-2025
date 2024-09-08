package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.hardware.caching.SolversCRServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;

@Config
public class CoaxialSwerveModule {
    private final SolversMotor motor;
    private final SolversCRServo servo;
    private final AnalogInput absoluteEncoder;

    // Pod rotation PIDF
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    private double podTargetHeading = 0;
    private double podHeading = 0;

    // Value of encoder when pod faces straight and when the motor runs forward the wheel also runs forward
    private final double encoderOffset;

    private boolean motorFlipped;
    private double motorTargetPower = 0;

    private final PIDFController podPIDF;

    public CoaxialSwerveModule(SolversMotor motor, SolversCRServo servo, AnalogInput absoluteEncoder, double encoderOffset) {
        this.motor = motor;
        this.servo = servo;
        this.absoluteEncoder = absoluteEncoder;
        this.encoderOffset = encoderOffset;
        podPIDF = new PIDFController(P, I, D, F);
        servo.setPwm(new PwmControl.PwmRange(500, 2500, 5000));
    }

    public void read() {
        podHeading = normalizeRadians((absoluteEncoder.getVoltage() / 3.3 * Math.PI*2) - encoderOffset);
    }

    public void update(double podTargetHeading, double motorTargetPower) {
        this.podTargetHeading = normalizeRadians(podTargetHeading);
        this.motorTargetPower = motorTargetPower;

        // Optimize with wheel flipping
        if (normalizeRadians(Math.abs(this.podTargetHeading - this.podHeading)) > Math.PI/2) {
            this.motorFlipped = true;
            this.podTargetHeading = normalizeRadians(this.podTargetHeading + Math.PI);
        } else {
            motorFlipped = false;
        }

        // Only for tuning purposes - remove once tuned pod PIDF or leave it :shrug:
        podPIDF.setPIDF(P, I, D, F);

        servo.setPower(podPIDF.calculate(podHeading, this.podTargetHeading));
        motor.setPower(motorFlipped ? -this.motorTargetPower : this.motorTargetPower);
    }
}
