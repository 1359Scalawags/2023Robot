// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.6; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.6; 

    public enum WheelPositions {
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight
    }
    public static final class DisplaySystem {
        public static final int PDHCANID = 1;
        public static final int CAM_WIDTH = 320;
        public static final int CAM_HEIGHT = 240;
        public static final int CAM_FPS = 15;
    }
    public static final class SwerveDrive {
        public static final class FrontLeft {
            public static final int DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
            public static final int STEER_MOTOR = 8; // FIXME Set front left module steer motor ID
            public static final int STEER_ENCODER = 17; // FIXME Set front left steer encoder ID   
            public static final double STEER_OFFSET = -Math.toRadians(282); // FIXME Measure and set front left steer offset            
        }
        public static final class FrontRight {
            public static final int DRIVE_MOTOR = 5; // FIXME Set front left module drive motor ID
            public static final int STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
            public static final int STEER_ENCODER = 15; // FIXME Set front left steer encoder ID
            public static final double STEER_OFFSET = -Math.toRadians(354.0); // FIXME Measure and set front left steer offset       
        }
        public static final class BackLeft {
            public static final int DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID
            public static final int STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
            public static final int STEER_ENCODER = 11; // FIXME Set front left steer encoder ID
            public static final double STEER_OFFSET = -Math.toRadians(127.0); // FIXME Measure and set front left steer offset
        }
        public static final class BackRight {
            public static final int DRIVE_MOTOR = 3; // FIXME Set front left module drive motor ID
            public static final int STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
            public static final int STEER_ENCODER = 13; // FIXME Set front left steer encoder ID
            public static final double STEER_OFFSET = -Math.toRadians(333.0+70); // FIXME Measure and set front left steer offset                 
        }
        public static final double motorSpeed = 0.3;
        public static final double gearRatio = 8.14;
        public static final double P = 0.00004;
        public static final double I = 0.00004;
        public static final double D = 0.00004;
    }

    public static final class Arm {
        public static final double armSpeedMultiplier = 0.1;
        public static final double armCoefficientMultiplier = 0.1;
        public static final double kP = 0.1;
        public static final double kI = 1e-4 ;
        public static final double kD = 1;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1; 
        public static final class Shoulder {
            public static final int channel = 1;                      
            public static final int motor = 30;  
            public static final double upperlimit = 65;
            public static final double lowerlimit = 125;
        }
        public static final class Elbow {
            public static final int channel = 2;        
            public static final int motor = 23;
            public static final double upperlimit = 265;
            public static final double lowerlimit = 120;
            
        }
    }

    public static final class Grabber {
        public static final int compressorModule =  21;        
        public static final int closedSolenoidModuleA = 0;
       // public static final int closedSolenoidModuleB = 1;
        public static final int openSolenoidModuleA = 1;
       // public static final int openSolenoidModuleB = 1;
        public static class Pneumatic {
            public static final int pistionOne = 1;                
            public static final int pistionTwo = 2;
        }
    }

    public static final class UI {
        public static final double deadband = 0.05;
    }
}
