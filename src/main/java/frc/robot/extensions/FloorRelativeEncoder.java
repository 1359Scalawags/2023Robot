package frc.robot.extensions;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class FloorRelativeEncoder implements Sendable {

    private double direction;
    private double offset;
    private DutyCycleEncoder encoder;
    private double lastRetrievedValue;
    private FloorRelativeEncoder relativeTo;

    /**
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
        relativeTo = null;
    }

    /**
     * Create an encoder where measurements are relative to the floor.
     * @param channel The digital channel to attach to.
     * @param angleAtFloor The native angle of the encoder.
     * @param reverse Does the encoder read angles backwards?
     * @param relativeTo The encoder of a parent arm.
     */
    public FloorRelativeEncoder(int channel, double angleAtFloor, boolean reversed, FloorRelativeEncoder relativeTo) {
        this(channel, angleAtFloor, reversed);
        this.relativeTo = relativeTo;
    }

    /**
     * Set offset based on current position of the arm.
     * Added as an example for future robots. No current use is anticipated.
     */
    public void setDynamicOffset() {
        this.offset = this.encoder.getAbsolutePosition() * 360;
    }  
    
    public boolean isAttachedToParent() {
        return (relativeTo != null);
    }
    
    /**
     * Get the position relative to the floor (in degrees).
     * @return An angle in degrees.
     */
    public double getDegrees() {
        this.lastRetrievedValue = (this.direction) * (this.encoder.getAbsolutePosition() * 360 - this.offset);
        if(relativeTo != null) {
            this.lastRetrievedValue = this.lastRetrievedValue + relativeTo.getDegrees();
        }
        return this.lastRetrievedValue;
    }

    /**
     * Get position when it was last requested.
     * @return A position in degrees relative to the floor.
     */
    public double getPriorReadingDegrees() {
        return this.lastRetrievedValue;
    }

    /**
     * Get the position relative to the floor (in radians).
     * @return An angle in radians.
     */
    public double getRadians() {
        return this.getDegrees() * Math.PI / 180.0;
    }

    /**
     * Get position when it was last requested.
     * @return A position in radians relative to the floor.
     */
    public double getPriorReadingRadians() {
        return this.lastRetrievedValue * Math.PI / 180.0;
    }



    /**
     * Get the offset from the native encoder measurement.
     * @return An angle in degrees.
     */
    public double getOffset() {
        return this.offset;
    }

    /**
     * Are we reversing the native encoder scale.
     * @return True, if the encoder scale is reversed.
     */
    public boolean isReversed() {
        return this.direction == -1;   
    }


    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Floor Relative Encoder");
        builder.addDoubleProperty("ϴ Deg", this::getDegrees, null);
        builder.addDoubleProperty("ϴ Rad", this::getRadians, null);
        builder.addDoubleProperty("ϴ Offset", this::getOffset, null);
        builder.addBooleanProperty("Reversed?", this::isReversed, null);
      }



}
