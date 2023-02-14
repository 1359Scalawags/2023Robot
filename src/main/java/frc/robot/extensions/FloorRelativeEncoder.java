package frc.robot.extensions;

import java.io.NotActiveException;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class FloorRelativeEncoder implements Sendable {

    private double direction;
    private double offset;
    private DutyCycleEncoder encoder;

    /***
     * Create an encoder where measurements are relative to the floor.
     * @param channel The digital channel to attach to.
     * @param angleAtFloor The native angle of the encoder.
     * @param reverse Does the encoder read angles backwards?
     */
    public FloorRelativeEncoder(int channel, double angleAtFloor, boolean reversed) {
        this.encoder = new DutyCycleEncoder(channel);
        this.encoder.setDistancePerRotation(360.0);
        this.offset = angleAtFloor;
        if(reversed)
            this.direction = -1;
        else 
            this.direction = 1;
    }

    /***
     * Get the position relative to the floor (in degrees).
     * @return An angle in degrees.
     */
    public double getDegrees() {
        return (this.direction) * (this.encoder.getAbsolutePosition() - this.offset);
    }

    /***
     * Get the position relative to the floor (in radians).
     * @return An angle in radians.
     */
    public double getRadians() {
        return this.getDegrees() * Math.PI / 180.0;
    }

    /***
     * Set offset based on current position of the arm.
     * Added as an example for future robots. No current use is anticipated.
     */
    public void setDynamicOffset() {
        this.offset = this.encoder.getAbsolutePosition();
    }

    public double getOffset() {
        return this.offset;
    }

    public boolean isReversed() {
        return this.direction == -1;   
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Floor Relative Encoder");
        builder.addDoubleProperty("Degrees", this::getDegrees, null);
        builder.addDoubleProperty("Radians", this::getRadians, null);
        builder.addDoubleProperty("Offset", this::getOffset, null);
        builder.addBooleanProperty("Reversed?", this::isReversed, null);
      }



}
