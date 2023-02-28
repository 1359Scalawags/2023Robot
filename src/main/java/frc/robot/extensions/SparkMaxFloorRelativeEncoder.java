// /**
//  * This class probably has no realistic use since we can simply
//  * update the zeroOffset of the SparkMax controllers as needed.
//  */



// package frc.robot.extensions;

// import com.revrobotics.SparkMaxAbsoluteEncoder;

// import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;


// public class SparkMaxFloorRelativeEncoder implements Sendable {

//     private SparkMaxAbsoluteEncoder encoder;
//     private SparkMaxFloorRelativeEncoder relativeTo;
//     private double angleAtFloor;
//     //private LinearFilter filter = LinearFilter.movingAverage(5);

//     /**
//      * Create an encoder where measurements are relative to the floor.
//      * @param channel The digital channel to attach to.
//      * @param angleAtFloor The native angle of the encoder.
//      * @param reverse Does the encoder read angles backwards?
//      */
//     public SparkMaxFloorRelativeEncoder(SparkMaxAbsoluteEncoder absoluteEncoder, double positionConversionFactor, double angleAtFloor, boolean reversed) {
//         this.encoder = absoluteEncoder;
//         this.encoder.setPositionConversionFactor(positionConversionFactor);        
//         this.encoder.setInverted(reversed);
//         this.encoder.setZeroOffset(angleAtFloor);
//         this.relativeTo = null;
//         this.angleAtFloor= angleAtFloor;
//     }

//     /**
//      * Create an encoder where measurements are relative to the floor.
//      * @param channel The digital channel to attach to.
//      * @param angleAtFloor The native angle of the encoder.
//      * @param reverse Does the encoder read angles backwards?
//      * @param relativeTo The encoder of a parent arm.
//      */
//     public SparkMaxFloorRelativeEncoder(SparkMaxAbsoluteEncoder absoluteEncoder, double positionConversionFactor, double angleAtFloor, boolean reversed, SparkMaxFloorRelativeEncoder relativeTo) {
//         this(absoluteEncoder, positionConversionFactor, angleAtFloor, reversed);
//         this.relativeTo = relativeTo;
//     }
    
//     public boolean isAttachedToParent() {
//         return (relativeTo != null);
//     }
    
//     /**
//      * Get the position relative to the floor (in degrees).
//      * @return An angle in degrees.
//      */
//     public double getDegrees() {
//         return this.encoder.getPosition();
//     }

//     // public double convertToAbsoluteDegrees(double floorRelativeAngle) {
//     //     return (floorRelativeAngle + this.offset) / (360.0*this.direction);
//     // }

//     /**
//      * Get the position relative to the floor (in radians).
//      * @return An angle in radians.
//      */
//     public double getRadians() {
//         return this.encoder.getPosition() * Math.PI / 180.0;
//     }

//     /**
//      * Get the offset from the native encoder measurement.
//      * @return An angle in degrees.
//      */
//     public double getOffset() {
//         return this.encoder.getZeroOffset();
//     }

//     /**
//      * Are we reversing the native encoder scale.
//      * @return True, if the encoder scale is reversed.
//      */
//     public boolean isReversed() {
//         return this.encoder.getInverted(); 
//     }

//     public void update() {
//         if(relativeTo != null) {
//             this.encoder.setZeroOffset(angleAtFloor + relativeTo.getDegrees());
//         } 
//     }

//     public void initSendable(SendableBuilder builder) {
//         builder.setSmartDashboardType("Floor Relative Encoder");
//         builder.addDoubleProperty("ϴ Deg", this::getDegrees, null);
//         builder.addDoubleProperty("ϴ Rad", this::getRadians, null);
//         builder.addDoubleProperty("ϴ Offset", this::getOffset, null);
//         builder.addBooleanProperty("Reversed?", this::isReversed, null);
//       }



// }
