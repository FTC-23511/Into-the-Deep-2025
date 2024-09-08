package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.ARM_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.ARM_TRANSFER_POS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.LEFT_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.LEFT_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.MAX_SLIDES_EXTENSION;
import static org.firstinspires.ftc.teamcode.hardware.Globals.RIGHT_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.RIGHT_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.WRIST_BACKDROP_POSITIONS;
import static org.firstinspires.ftc.teamcode.hardware.Globals.WRIST_TRANSFER_POS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Deposit extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private static final PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public int pixelHeight = 1;
    public int wristIndex = 3;
    public boolean wristTransfer;
    private double target;

    // Between transfer and backdrop position
    public boolean armTransfer;

    // Between retracted and extended
    public boolean slidesRetracted;

    // Between open and closed
    public boolean leftClawOpen;
    public boolean rightClawOpen;

    // Default will reset deposit to transfer position (unpowered claw servos depending on auto vs tele-op)
    public void init() {
        slidePIDF.setTolerance(10, 10);

        setArmTransfer(true);

        setWristTransfer();

        setSlideTarget(0);
    }

    public void initAuto() {
        setClaw(false, false);
    }

    public void initTeleOp() {
        setClaw(true, true);
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_SLIDES_EXTENSION), 0);
        slidePIDF.setSetPoint(target);
    }

    public void autoSetSlidePower() {
        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftRight.setPower(power);
        robot.liftLeft.setPower(power);

        // Slides are only retracted once stopped and at a target of 0
        slidesRetracted = ((target <= 0)) && (this.reached());
    }

    // Returns if slides have reached the target
    public boolean reached() {
        return (slidePIDF.atSetPoint());
    }

    public void setArmTransfer(boolean armTransfer) {
        robot.rightArm.setPosition(armTransfer ? ARM_TRANSFER_POS : ARM_BACKDROP_POS);
        robot.leftArm.setPosition(armTransfer ? -ARM_TRANSFER_POS + 1 : -ARM_BACKDROP_POS + 1);
        this.armTransfer = armTransfer;
    }

    // Be careful with these 2 methods to make sure armState is at the relevant state/position
    public void moveWrist() {
        robot.wrist.setPosition(WRIST_BACKDROP_POSITIONS[Math.max(Math.min(wristIndex, 5), 0)]);
        wristTransfer = false;
    }

    public void teleOpSetClaw(boolean leftClawOpen, boolean rightClawOpen) {
        if (wristIndex == 0 || wristIndex == 1 || wristIndex == 5) { // Right Claw is now Left Claw
            setClaw(rightClawOpen, leftClawOpen);
        } else {
            setClaw(leftClawOpen, rightClawOpen);
        }
    }

    public void setWristTransfer() {
        robot.wrist.setPosition(WRIST_TRANSFER_POS);
        wristTransfer = true;
    }

    public void setClaw(boolean leftClawOpen, boolean rightClawOpen) {
        if (leftClawOpen) {
            robot.leftClaw.setPosition(LEFT_CLAW_OPEN_POS);
        } else {
            robot.leftClaw.setPosition(LEFT_CLAW_CLOSE_POS);
        }
        this.leftClawOpen = leftClawOpen;

        if (rightClawOpen) {
            robot.rightClaw.setPosition(RIGHT_CLAW_OPEN_POS);
        } else {
            robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE_POS);
        }
        this.rightClawOpen = rightClawOpen;
    }

    @Override
    public void periodic() {
        autoSetSlidePower();
    }
}
