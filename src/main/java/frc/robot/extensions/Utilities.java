package frc.robot.extensions;

public class Utilities {
    public static boolean IsCloseTo(double a, double b, double tolerance) {
        if(Math.abs(a-b) < tolerance) {
            return true;
        }
        return false;
    }

    public static double getSignedSquare(double value) {
        return Math.signum(value) * Math.pow(value, 2);
    }

}
