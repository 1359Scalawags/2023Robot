// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.RelativeEncoder;
// import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
// import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.SetDriveMode;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Constants.SwerveDrive.*;
import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    /**
     * Get the current instance of the drivetrain used on the robot.
     */
    private static DrivetrainSubsystem instance;

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    public static final double MAX_VOLTAGE = 12.0;
    public enum DriveModes {
        RobotCentric,
        FieldCentric
      }
    private DriveModes driveMode;
    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_MPS = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_MPS /
        Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    private static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    private SwerveDriveOdometry m_Odometry;
    private Pose2d m_pose;
    // private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final PIDController xController = new PIDController(Constants.kp_XController, Constants.ki_XController, Constants.kd_XController);
    private final PIDController yController = new PIDController(Constants.kp_YController, Constants.ki_YController, Constants.kd_YController);
    private final PIDController rotationController = new PIDController(Constants.kp_RotationController, Constants.ki_RotationController, Constants.kd_RotationController);
    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


    public DrivetrainSubsystem() {
        driveMode = DriveModes.RobotCentric;
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
// TODO: Setup motor configuration
        // m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        //     // See the current state of the module on the dashboard.
        //     tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        //             .withSize(2, 4)
        //             .withPosition(0, 0),
        //     // This can either be STANDARD or FAST depending on your gear configuration
        //     Mk4iSwerveModuleHelper.GearRatio.L1,
        //     FrontLeft.DRIVE_MOTOR,
        //     FrontLeft.STEER_MOTOR,
        //     FrontLeft.STEER_ENCODER,
        //     FrontLeft.STEER_OFFSET);

        m_frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, FrontLeft.DRIVE_MOTOR)
            .withSteerMotor(MotorType.FALCON, FrontLeft.STEER_MOTOR)
            .withSteerEncoderPort(FrontLeft.STEER_ENCODER)
            .withSteerOffset(FrontLeft.STEER_OFFSET)
            .build();

        // TODO: Change Mk4iSwerveModuleHelper to MkSwerveModuleBuilder
        // We will do the same for the other modules
        m_frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0))
                    .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                    .withDriveMotor(MotorType.FALCON, FrontRight.DRIVE_MOTOR)
                    .withSteerMotor(MotorType.FALCON, FrontRight.STEER_MOTOR)
                    .withSteerEncoderPort(FrontRight.STEER_ENCODER)
                    .withSteerOffset(FrontRight.STEER_OFFSET)
                    .build();


        /*m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),

            // TODO get .PID.Controller(),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BackLeft.DRIVE_MOTOR,
            BackLeft.STEER_MOTOR,
            BackLeft.STEER_ENCODER,
            BackLeft.STEER_OFFSET);*/
        m_backLeftModule = new MkSwerveModuleBuilder()
             .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.FALCON, BackLeft.DRIVE_MOTOR)
                        .withSteerMotor(MotorType.FALCON, BackLeft.STEER_MOTOR)
                        .withSteerEncoderPort(BackLeft.STEER_ENCODER)
                        .withSteerOffset(BackLeft.STEER_OFFSET)
                        .build();



        /*m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BackRight.DRIVE_MOTOR,
            BackRight.STEER_MOTOR,
            BackRight.STEER_ENCODER,
            BackRight.STEER_OFFSET);*/

        m_backRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.FALCON, BackRight.DRIVE_MOTOR)
                        .withSteerMotor(MotorType.FALCON, BackRight.STEER_MOTOR)
                        .withSteerEncoderPort(BackRight.STEER_ENCODER)
                        .withSteerOffset(BackRight.STEER_OFFSET)
                        .build();

        // set the current instance as the public instance
        instance = this;

        m_Odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), new SwerveModulePosition[]
        {
            m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(), m_backRightModule.getPosition()
        }, new Pose2d(kMaxAccelerationMetersPerSecondSquared, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, new Rotation2d()));

    }


    /**
     * Access the modules individually for testing
     */
    public SwerveModule getModule(WheelPositions position) {
        if (position == WheelPositions.FrontLeft)
            return m_frontLeftModule;
        else if (position == WheelPositions.FrontRight)
            return m_frontRightModule;
        else if (position == WheelPositions.BackLeft)
            return m_backLeftModule;
        else if (position == WheelPositions.BackRight)
            return m_backRightModule;
        else
            return null;

    }

    public void setModule(WheelPositions position, double speed, double degrees) {
        double radians = degrees * Math.PI / 180;
        switch (position) {
            case FrontLeft:
                m_frontLeftModule.set(speed / MAX_VELOCITY_MPS * MAX_VOLTAGE, radians);
                break;
            case FrontRight:
                m_frontRightModule.set(speed / MAX_VELOCITY_MPS * MAX_VOLTAGE, radians);
                break;
            case BackLeft:
                m_backLeftModule.set(speed / MAX_VELOCITY_MPS * MAX_VOLTAGE, radians);
                break;
            case BackRight:
                m_backRightModule.set(speed / MAX_VELOCITY_MPS * MAX_VOLTAGE, radians);
                break;
            default:

        }

    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {

        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }
        //
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    /**
     * Returns the yaw of the robot.
     * @return The yaw angle from -180 to 180 degrees.
     */
    public double getYawAsDegrees() {
        return m_navx.getYaw();
        //return getGyroscopeRotation().getDegrees();
    }
    
    public double getYawAsRadians() {
        return Rotation2d.fromDegrees(m_navx.getYaw()).getRadians();
    }

    public double getPitch() {
        return m_navx.getPitch();
        //return getGyroscopeRotation().getDegrees();
    }

    public double getDistanceX() {
        return m_navx.getDisplacementX();
    }

    public double getDistanceY() {
        return m_navx.getDisplacementY();
    }

    /**
     * Set translation and rotation speeds of the drive.
     * @param chassisSpeeds Speeds to use on next update of drive.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void setDriveMode(DriveModes mode) {
        driveMode = mode;
    }

    /**
     * Get the translation and rotation speeds of the drive.
     * @return A ChassisSpeeds object.
     */
    public ChassisSpeeds getSpeeds() {
        return m_chassisSpeeds;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(traj, 
        this::getPose, 
        m_kinematics, 
        xController,
        yController,
        rotationController, 
        this::setModuleStates, 
        isFirstPath, 
        this);
        return new SequentialCommandGroup(  
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   m_Odometry.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[] {
                    m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                    m_backLeftModule.getPosition(), m_backRightModule.getPosition()
                   },traj.getInitialHolonomicPose());
               }
             }), swerveCommand
         );
     }

     public void setModuleStates(SwerveModuleState[] states){
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[3].angle.getRadians());
     }


    @Override
    public void periodic() {

        m_pose = m_Odometry.update(getGyroscopeRotation(),  new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(), m_backRightModule.getPosition()
          });

        if (!Robot.isTestMode()) {
            ChassisSpeeds tempSpeeds = m_chassisSpeeds;
            if (driveMode == DriveModes.FieldCentric){
                tempSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeeds, getGyroscopeRotation()); 
            }

            SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(tempSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_MPS);

            m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[0].angle.getRadians());
            m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[1].angle.getRadians());
            m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[2].angle.getRadians());
            m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE,
                    states[3].angle.getRadians());
        }

    }

}

        

