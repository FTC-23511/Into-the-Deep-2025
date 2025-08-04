package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum HangState {
        RETRACT,
        EXTEND,
        STOP
    }

    public enum SubPusherState {
        IN,
        OUT,
        AUTO_PUSH
    }

    public static HangState hangState;
    public static SubPusherState subPusherState = SubPusherState.IN;

    public void init() {
        setHang(HangState.STOP);
        setSubPusher(SubPusherState.IN);
    }

    public void setHang(HangState hangState) {
        switch (hangState) {
            case RETRACT:
                robot.leftHang.setPower(-LEFT_HANG_FULL_POWER);
                robot.rightHang.setPower(-RIGHT_HANG_FULL_POWER);
                break;
            case EXTEND:
                robot.leftHang.setPower(LEFT_HANG_FULL_POWER);
                robot.rightHang.setPower(RIGHT_HANG_FULL_POWER);
                break;
            case STOP:
                robot.leftHang.setPower(0);
                robot.rightHang.setPower(0);
                break;
        }

        Drive.hangState = hangState;
    }

    public void setSubPusher(SubPusherState subPusherState) {
        switch (subPusherState) {
            case IN:
                robot.subPusher.setPosition(SUB_PUSHER_IN);
                break;
            case OUT:
                robot.subPusher.setPosition(SUB_PUSHER_OUT);
                break;
            case AUTO_PUSH:
                robot.subPusher.setPosition(SUB_PUSHER_AUTO);
                break;
        }

        Drive.subPusherState = subPusherState;
    }
}
