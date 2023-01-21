// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.SwerveDrive.*;
import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  private static DrivetrainSubsystem instance;
  
  public static DrivetrainSubsystem getInstance() {
        return instance;
  }


public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_MPS = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_MPS /
          Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  //private final PigeonIMU m_pigeon = new PigeonIMU(PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // 
    
    
    /****************************************
    IMPORTANT NOTE: Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.
    ****************************************/ 


    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FrontLeft.DRIVE_MOTOR,
            FrontLeft.STEER_MOTOR,
            FrontLeft.STEER_ENCODER,
            FrontLeft.STEER_OFFSET
    );


    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            
            FrontRight.DRIVE_MOTOR,
            FrontRight.STEER_MOTOR,
            FrontRight.STEER_ENCODER,
            FrontRight.STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
                    
                    //TODO get .PID.Controller(),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BackLeft.DRIVE_MOTOR,
            BackLeft.STEER_MOTOR,
            BackLeft.STEER_ENCODER,
            BackLeft.STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BackRight.DRIVE_MOTOR,
            BackRight.STEER_MOTOR,
            BackRight.STEER_ENCODER,
            BackRight.STEER_OFFSET
    );
    instance = this;
  }

  /**
   * Access the modules individually for testing
   */
  public SwerveModule getModule(WheelPositions position) {
    if(position == WheelPositions.FrontLeft)
      return m_frontLeftModule;
    else if(position == WheelPositions.FrontRight)
      return m_frontRightModule;
    // TODO: need the other cases
    else 
      return null;

  }

  public void setModule(WheelPositions position, double speed, double degrees) {
        double radians = degrees * Math.PI / 180;
        switch(position) {
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
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    //m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
    //return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    // FIXME Uncomment if you are using a NavX
    if (m_navx.isMagnetometerCalibrated()) {
//      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
          }
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public double getYaw() {
        return m_navx.getYaw();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    if(!Robot.isTestMode()){
       SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_MPS);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_MPS * MAX_VOLTAGE, states[3].angle.getRadians());     
    }
    
  }
 
}
