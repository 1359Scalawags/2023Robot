package org.scalawags.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
