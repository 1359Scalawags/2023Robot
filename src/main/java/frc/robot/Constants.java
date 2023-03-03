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
            public static final int DRIVE_MOTOR = 7; 
            public static final int STEER_MOTOR = 8; 
            public static final int STEER_ENCODER = 17; 
            public static final double STEER_OFFSET = -Math.toRadians(282 - 23); // FIXME Measure and set offset             
        }
        public static final class FrontRight {
            public static final int DRIVE_MOTOR = 5; 
            public static final int STEER_MOTOR = 6; 
            public static final int STEER_ENCODER = 15; 
            public static final double STEER_OFFSET = -Math.toRadians(354.0-1); // FIXME Measure and set offset       
        }
        public static final class BackLeft {
            public static final int DRIVE_MOTOR = 1; 
            public static final int STEER_MOTOR = 2; 
            public static final int STEER_ENCODER = 11; 
            public static final double STEER_OFFSET = -Math.toRadians(127.0); // // FIXME Measure and set offset   
        }
        public static final class BackRight {
            public static final int DRIVE_MOTOR = 3; 
            public static final int STEER_MOTOR = 4; 
            public static final int STEER_ENCODER = 13; 
            public static final double STEER_OFFSET = -Math.toRadians(333.0+70-3); // FIXME Measure and set offset                 
        }
        public static final double motorSpeed = 0.5;
        public static final double gearRatio = 8.14;
        public static final double rotateMultiplier = 0.6;

        //FIXME: Are these being used?
        public static final double P = 0.00004;
        public static final double I = 0.00004;
        public static final double D = 0.00004;
    }

    public static final class Arm {
        //public static final double radtoDegreeDivisor = Math.PI / 180.0; // name did not match conversion
        public static final double boundaryExtension = 5;
        //public static final double ffTestRatio = 0.1; //split to each joint
        public static final double softStartRatio = 0.5;
        public static final class Shoulder {
            //public static final double ffTestRatio = 0.75;   
            //public static final double pidTestRatio = 1; //30 // 14.25 sec for oscilation       
            public static final int channel = 1;                      
            public static final int motor = 30;  
            public static final double upperlimit = 265.0;
            public static final double lowerlimit = 200.0;
            public static final double defaultSetpoint = 225.0;
            public static final double CoefficientMultiplier = 1/360.0;
            public static final double shoulderSpeedMultiplier = 0.85;
            // public static final double kP = 60 * pidTestRatio;  // Aft
            // public static final double kI = 15 * pidTestRatio;// 0.12 * pidTestRatio;  //10e-4
            // public static final double kD = 7 * pidTestRatio;//0.001875 * pidTestRatio;  //1
            // public static final double kS = 0.10 * ffTestRatio;
            // public static final double kG = 0.47 * ffTestRatio;
            // public static final double kV = 1.19 * radtoDegreeDivisor;
            // public static final double kA = 0.07 * radtoDegreeDivisor * ffTestRatio;            
            
            public static final double kP = 0.0087;
            public static final double kPAuto = 0.0083; 
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.00026;
            public static final double kIz = 0;
            public static final double kMinOutput = -0.25;
            public static final double kMaxOutput = 0.25;

            public static final double kS = 0.075;
            public static final double kG = 0.3525;
            public static final double kV = 0.020769;
            public static final double kA = 0.000916;

            public static final double angleAtFloor = 213.0 - 180.0;
            public static final double targetSpeed = 0.05;
            public static final double maxVoltage = 3;
            public static final double minVoltage = -maxVoltage;
            // public static final double kIz = 0;
            // public static final double kFF = 0;
            // public static final double kMaxOutput = 1;
            // public static final double kMinOutput = -1; 

            public static final double parkingDegree = 270.0;
            public static final double onGroundLevel = 226;
            public static final double onHighLevel = 211;
            public static final double onMidLevel = 240;
            public static final double onSubStation = 255;

            public static final double tolerance = 10.0;
            public static final double slewRateLimiter = 1;
        }
        public static final class Elbow {
            //public static final double ffTestRatio = 0.8;
            //public static final double pidTestRatio = 0.2;  
            public static final int channel = 2;        
            public static final int motor = 23;
            public static final double upperlimit = 215.0;
            public static final double lowerLimitMax = 115;
            public static final double lowerLimitMin = 85.0;
            public static final double shoulderRestrictionPositionLower = 220;
            public static final double shoulderRestrictionPositionUpper = 270;
            public static final double defaultSetpoint = 104.0;
            public static final double CoefficientMultiplier = 1/360.0;
            public static final double elbowSpeedMultiplier = 0.85; //TODO: Tune This... I don't know how much to increase it by
            //(1/360.0);
            // public static final double kP = 0.03 * pidTestRatio;  // After tuning, 0.05/2
            // public static final double kI = 0.06 * pidTestRatio;  //10e-4
            // public static final double kD = 0.00375 * pidTestRatio;  //1 //1
            // public static final double kS = 0.15 * ffTestRatio; // cut by 20% for initial testing
            // public static final double kG = 0.19 * ffTestRatio;
            // public static final double kV = 1.17 * radtoDegreeDivisor;
            // public static final double kA = 0.02 * radtoDegreeDivisor * ffTestRatio ;
            
            public static final double kP = 0.01; //0.18;  // After tuning, 0.05/2
            public static final double kPAuto = 0.009;
            public static final double kI = 0;  //10e-4
            public static final double kD = 0;  //1 
            public static final double kFF = 0.0001;
            public static final double kIz = 0;
            public static final double kMinOutput = -0.2;
            public static final double kMaxOutput = 0.2;

            public static final double kS = 0.11; // cut by 20% for initial testing
            public static final double kG = 0.15;
            public static final double kV = 0.020420352;
            public static final double kA = 0.000279253;

            public static final double angleAtFloor = 27.0;
            public static final double targetSpeed = 0.05;
            public static final double maxVoltage = 3; // based on 12V
            public static final double minVoltage = -maxVoltage; // FIXME: this isn't necessary to store in memory
            // public static final double kIz = 0;
            // public static final double kFF = 0;
            // public static final double kMaxOutput = 1;
            // public static final double kMinOutput = -1; 

            public static final double parkingDegree = 115.0;
            public static final double onGroundLevel = 115;
            public static final double onHighLevel = 211;
            public static final double onMidLevel = 184;
            public static final double onSubStation = 200;

            public static final double tolerance = 5.0;
            public static final double slewRateLimiter = 1;
        }
    }

    public static final class Grabber {
        public static final int compressorModule =  21;        
        public static final int closedSolenoidModuleA = 0;
       // public static final int closedSolenoidModuleB = 1;
        public static final int openSolenoidModuleA = 1;
       // public static final int openSolenoidModuleB = 1;
        public static class Pneumatic {
            public static final int PneumaticHub = 21;
            public static final int pistionOne = 1;                
            public static final int pistionTwo = 2;
            public static int forwardChannel =0;
            public static int reverseChannel =1;
        }
    }

    public static final class Autonomous {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double autoSpeed = 2;

    }

    public static final class UI {
        public static final double deadband = 0.05;
    }
}
