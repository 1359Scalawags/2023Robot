// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
     * The left-to-right distance between the drivetrain wheels (center to center)
     */
    public static final double TRACKWIDTH_METERS = 0.6; 
    
    /**
     * The front-to-back distance between the drivetrain wheels (center to center).
     */
    public static final double WHEELBASE_METERS = 0.6;     
    
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TRACKWIDTH_METERS);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Flags for displaying or hiding shuffleboard widgets at deployment
    public static final boolean SHOW_DEBUG_WIDGETS = true;
    
    //What are these for?
    public static final double ksVolts = 0.12778;
    public static final double kvVoltSecondsPerMeter = 2.8444; 
    public static final double kaVoltSecondsSquarePerMeter = 0.20256;
    public static final double kPDriveVel = 0.17182;
    
    public static final double kp_XController = 0.831985;
    public static final double ki_XController = 0.0;
    public static final double kd_XController = 0;
    public static final double kp_YController = 0.831985;
    public static final double ki_YController = 0.0;
    public static final double kd_YController = 0;
    public static final double kp_RotationController = 0.831985;
    public static final double ki_RotationController = 0.0;
    public static final double kd_RotationController = 0.07;
    // public static final double kp_RotationController = 2; //1.57;
    // public static final double ki_RotationController = 0.0;     
    // public static final double kd_RotationController = 0.07; //0.02;
    
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
            public static final double STEER_OFFSET = -Math.toRadians(282 - 23 - 4.5 + 3);           
        }
        public static final class FrontRight {
            public static final int DRIVE_MOTOR = 5; 
            public static final int STEER_MOTOR = 6; 
            public static final int STEER_ENCODER = 15; 
            public static final double STEER_OFFSET = -Math.toRadians(354.0-1 + 4.5);      
        }
        public static final class BackLeft {
            public static final int DRIVE_MOTOR = 1; 
            public static final int STEER_MOTOR = 2; 
            public static final int STEER_ENCODER = 11; 
            public static final double STEER_OFFSET = -Math.toRadians(127); 
        }
        public static final class BackRight {
            public static final int DRIVE_MOTOR = 3; 
            public static final int STEER_MOTOR = 4; 
            public static final int STEER_ENCODER = 13; 
            public static final double STEER_OFFSET = -Math.toRadians(333.0+70-3);                
        }
        public static final double motorSpeed = 0.5;
        public static final double gearRatio = 8.14;
        public static final double rotateMultiplier = 0.6;

        //Are these being used?
        public static final double P = 0.00004;
        public static final double I = 0.00004;
        public static final double D = 0.00004;
    }

    public static final class Vision {
        public static final double limelightMountAngleDegree = 0;
        public static final double limelightLensHeightInches = 0;
        public static final double goalHeightInches = 0;
    }

    public static final class Arm {

        public static final double triggerZone = 15;

        public static final double parkingTolerance = 5.0;
        public static final double boundaryExtension = 5;
        public static final double softStartRatio = 0.5;
        
        public static final class Shoulder {
            public static final int channel = 1;                      
            public static final int motor = 30;  
            public static final double upperlimit = 269;
            public static final double lowerlimit = 215.0;
            public static final double safelimit = 260.0;
            public static final double defaultSetpoint = 268.0;
            public static final double shoulderSpeedMultiplier = 0.85;
              
            public static final double kP = 0.009;
            // public static final double kPAuto = 0.0083; 
            public static final double kI = 0.0000015;
            public static final double kD = 0.0000004;
            public static final double kFF = 0.;
            public static final double kIz = 0;

            //FIXME: Is this preventing shoulder from working properly?
            public static final double kMinOutput = -0.3;
            public static final double kMaxOutput = 0.5;

            public static final double kS = 0.075;
            public static final double kG = 0.3525;
            public static final double kV = 0.020769;
            public static final double kA = 0.000916;

            public static final double kGravityAssistFF_Max = 0.0008;  //when arm parallel to floor
            public static final double kGravityAssistFF_Min = 0.000001;  //when arm perpendicular to floor

            public static final double angleAtFloor = 213.0 - 180.0;
            public static final double targetSpeed = 0.05;
            public static final double maxVoltage = 3;
            public static final double minVoltage = -maxVoltage;

            public static final double unParkingDegree = 267;
            public static final double parkingDegree = upperlimit;
            public static final double onGroundLevel = 265;
            public static final double onHighLevel = 220;
            public static final double onMidLevel = 253;
            public static final double onSubStation = 256;

            public static final double tolerance = 5.0;
            public static final double slewRateLimiter = 30.0;  // 20.0 * 1.5
        }
        public static final class Elbow {
            public static final int channel = 2;        
            public static final int motor = 23;
            public static final double upperlimit = 220.0;
            public static final double lowerLimitWhenShoulderSafe = 130;
            public static final double lowerLimitWhenSafePos = 130;
            public static final double lowerLimitUnsafePosMin = 130;
            public static final double shoulderRestrictionPositionLower = 220;
            public static final double shoulderRestrictionPositionUpper = 270;
            public static final double defaultSetpoint = 130.0;
            public static final double elbowSpeedMultiplier = 1; 
            
            public static final double kP = 0.01; //0.18;  // After tuning, 0.05/2
            public static final double kPAuto = 0.009;
            public static final double kI = 0.00001;  //10e-4
            public static final double kD = 0.000004;  //1 
            public static final double kFF = 0.0001;
            public static final double kIz = 0;

            //FIXME: Is this preventing the arm from working properly?
            public static final double kMinOutput = -0.3;
            public static final double kMaxOutput = 0.3;

            public static final double kS = 0.11; // cut by 20% for initial testing
            public static final double kG = 0.15;
            public static final double kV = 0.020420352;
            public static final double kA = 0.000279253;

            public static final double angleAtFloor = 27.0 - 60;
            public static final double targetSpeed = 0.05;
            public static final double maxVoltage = 3; // based on 12V
            public static final double minVoltage = -maxVoltage; // FIXME: this isn't necessary to store in memory

            public static final double unParkingDegree = 135;
            public static final double parkingDegree = 125;
            public static final double onGroundLevel = 136;
            public static final double onHighLevel = 220;
            public static final double onMidLevel = 176;
            public static final double onSubStation = 179;

            public static final double tolerance = 5.0;
            public static final double slewRateLimiter = 90;  // 60.0 * 1.5

            public static final class Limiter {
                public static final class Upper {
                    public static final double m = -1;
                    public static final double b = 440;
                }
                public static final class Lower {
                    public static final double a = 0.0000033922;
                    public static final double b = -1.46987;  
                    public static final double c = 444.188;                  
                }
            }
        }
    }

    public static final class Grabber {
        public static final int compressorModule =  21;        
        public static final int closedSolenoidModuleA = 0;
        public static final int closedSolenoidModuleB = 1;
        public static final int openSolenoidModuleA = 1;
        public static final int openSolenoidModuleB = 3;
        public static class Pneumatic {
            public static final int PneumaticHub = 21;
            public static final int pistionOne = 1;                
            public static final int pistionTwo = 2;
            public static int forwardChannel = 0;
            public static int reverseChannel = 1;
        }
    }

    public static final class Autonomous {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double autoSpeed = 1.25;
        public static final double stopTime = 4;
        public static final double degreeOffset = 4;
        public static final double offsetFromCenter = 0;
        public static final double speedMultiplier = 0.01;

    }

    public static final class UI {
        public static final double deadband = 0.05;
        public static final int delayCounter = 7;  // was 5.0
    }

}
