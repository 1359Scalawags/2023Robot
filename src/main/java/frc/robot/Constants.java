// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TRACKWIDTH_METERS);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.6; 

    public static final double ksVolts = 0.12778;
    public static final double kvVoltSecondsPerMeter = 2.8444; 
    public static final double kaVoltSecondsSquarePerMeter = 0.20256;
    public static final double kPDriveVel = 0.17182;

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
        public static final class Pneumatic {
            public static final int pistionOne = 1;                
            public static final int pistionTwo = 2;                
        } 
            public static final double motorSpeed = 0.3;
            public static final double gearRatio = 8.14;

            public static final double P = 0.00004;
            public static final double I = 0.00004;
            public static final double D = 0.00004;
    }
}
