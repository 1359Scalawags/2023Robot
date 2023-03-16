package frc.robot.extensions;

public class Utilities {
    public static boolean IsCloseTo(double a, double b, double tolerance) {
        if(Math.abs(a-b) < tolerance) {
            return true;
        }
        return false;
    }

}
