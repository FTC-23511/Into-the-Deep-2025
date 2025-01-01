package com.seattlesolvers.ftclib.hardware;


import com.seattlesolvers.ftclib.geometry.Rotation2d;

public abstract class GyroEx implements HardwareDevice {


    public abstract void init();

    // Gyro
    public abstract double getHeading();

    public abstract double getAbsoluteHeading();

    public abstract double[] getAngles();

    public abstract Rotation2d getRotation2d();

    public abstract void reset();

}
