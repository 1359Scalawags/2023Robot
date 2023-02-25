package org.scalawags.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);
}
