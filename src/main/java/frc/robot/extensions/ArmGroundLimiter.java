package frc.robot.extensions;

public class ArmGroundLimiter {

    public class ArmSegment {
        private double length;
        private double floorAngleOffset;

        /**
         * Create an arm segment.
         * @param length The length of the arm from pivot to pivot (or pivot to tip).
         * @param horizontalOffsetAngle The angle where the arm is horizontal to the ground.
         */
        public ArmSegment(double length, double horizontalOffsetAngle) {
            this.length = length;
            this.floorAngleOffset = horizontalOffsetAngle;
        }

        /**
         * Gets the horizontal and vertical components of the arm.
         * @param rawAngle The angle in degrees as reported by the encoder.
         * @return The x and y components in the same units as the arm length.
         */
        public double getHorizontalComponent(double rawAngle) {
            double radianAngle = Math.toRadians(rawAngle - floorAngleOffset);
            return  this.length * Math.cos(radianAngle);
        }

        public double getVerticalComponent(double rawAngle) {
            double radianAngle = Math.toRadians(rawAngle - floorAngleOffset);           
            return this.length * Math.sin(radianAngle);
        }
    }    
    
    private ArmSegment primary;
    private ArmSegment secondary;
    private double mountHeight;
    private double groundBuffer;

    public ArmGroundLimiter(double mountHeight, double groundBuffer, ArmSegment primaryArm, ArmSegment secondaryArm) {
        this.primary = primaryArm;
        this.secondary = secondaryArm;
        this.mountHeight = mountHeight;
        this.groundBuffer = groundBuffer;
    }

    public double getPrimaryLowerLimit(double secondaryRawAngle) {
        double secondaryTotalHeight = this.secondary.getVerticalComponent(secondaryRawAngle);
        // lower limit will occur when secondary is pointed downward (negative angle)
        double targetHeight = -secondaryTotalHeight - this.mountHeight + this.groundBuffer;
        double targetAngle = Math.asin(targetHeight);
        return Math.toDegrees(targetAngle);
    }

    public double getSecondaryLowerLimit(double primaryRawAngle) {
        double primaryTotalHeight = this.primary.getVerticalComponent(primaryRawAngle) + this.mountHeight;
        // lower limit will have secondary pointed downward (negative angle)
        double targetHeight = -primaryTotalHeight + this.groundBuffer;
        double targetAngle = Math.asin(targetHeight);
        return Math.toDegrees(targetAngle);
    }
    
}
