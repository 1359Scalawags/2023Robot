// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.concurrent.ThreadPoolExecutor;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.WheelPositions;
import frc.robot.commands.DrivetrainNoAction;
import frc.robot.commands.SetDriveMode;
import frc.robot.commands.TurnWheelToAngleCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DisplaySubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;
import frc.robot.commands.ArmParkingCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GrabCommandClose;
import frc.robot.commands.GrabCommandOpen;
import frc.robot.commands.InitializeTargetRotationCommand;
import frc.robot.commands.PlatformBalance;
import frc.robot.commands.TurnCompressorOn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.commands.TurnCompressorOff;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSystem m_VisionSystem = new VisionSystem();
  private final DisplaySubsystem m_DisplaySubsystem = new DisplaySubsystem(m_VisionSystem);
  private final ZeroGyroCommand m_ZeroGyroCommand = new ZeroGyroCommand(m_drivetrainSubsystem);
  //private final XboxController m_controller = new XboxController(0);
  SendableChooser<Command> chooser = new SendableChooser<>();
  PathConstraints constraints = new PathConstraints(1.3, 0.5);
  PathPlannerTrajectory straightPath = PathPlanner.loadPath("Test Forward", constraints);
  PathPlannerTrajectory curvyPath = PathPlanner.loadPath("Curvy", constraints);
  // PathPlannerTrajectory TestPath = PathPlanner.loadPath("Test Forward", constraints);
  PathPlannerTrajectory BlueCSOnePath = PathPlanner.loadPath("BlueCS1", constraints);
  PathPlannerTrajectory BlueCSTwoPath = PathPlanner.loadPath("BlueCS2", constraints);
  PathPlannerTrajectory BlueCSThreePath = PathPlanner.loadPath("BlueCS3", constraints);
  PathPlannerTrajectory RedCSOnePath = PathPlanner.loadPath("RedCS1", constraints);
  PathPlannerTrajectory RedCSTwoPath = PathPlanner.loadPath("RedCS2", constraints);
  PathPlannerTrajectory RedCSThreePath = PathPlanner.loadPath("RedCS3", constraints);
  // private final PlatformBalance m_PlatformBalance = new PlatformBalance(m_drivetrainSubsystem);
  //  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  // private final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
  // private final TurnCompressorOff m_compressorOff = new TurnCompressorOff(m_grabberSubsystem);
  // private final TurnCompressorOn m_compressorOn = new TurnCompressorOn(m_grabberSubsystem);
  // private final GrabCommandOpen m_opengrabber = new GrabCommandOpen(m_grabberSubsystem);
  // private final GrabCommandClose m_closegrabber = new GrabCommandClose(m_grabberSubsystem); 
  // private final ArmParkingCommand m_ArmParkingCommand = new ArmParkingCommand(m_armSubsystem);

  //private final XboxController m_controller = new XboxController(0);
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick assistantJoystick = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    chooser.addOption("Straight path", m_drivetrainSubsystem.followTrajectoryCommand(straightPath, true));
    chooser.addOption("Curvy path", m_drivetrainSubsystem.followTrajectoryCommand(curvyPath, true));
    chooser.addOption("BlueCS1 path", m_drivetrainSubsystem.followTrajectoryCommand(BlueCSOnePath, true));
    chooser.addOption("BlueCS2 path", m_drivetrainSubsystem.followTrajectoryCommand(BlueCSTwoPath, true));
    chooser.addOption("BlueCS3 path", m_drivetrainSubsystem.followTrajectoryCommand(BlueCSThreePath, true));
    chooser.addOption("RedCS1 path", m_drivetrainSubsystem.followTrajectoryCommand(RedCSOnePath, true));
    chooser.addOption("RedCS2 path", m_drivetrainSubsystem.followTrajectoryCommand(RedCSTwoPath, true));
    chooser.addOption("RedCS3 path", m_drivetrainSubsystem.followTrajectoryCommand(RedCSThreePath, true));

    Shuffleboard.getTab("Autonomous").add(chooser);

    // Set up the default command for the drivetrain.s
    // m_grabberSubsystem.TurnOn(); 

    //m_grabberSubsystem.TurnOff();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      
            m_drivetrainSubsystem,
            () -> -modifyAxis(driverJoystick.getY(), driverJoystick.getThrottle()) * DrivetrainSubsystem.MAX_VELOCITY_MPS,
            () -> -modifyAxis(driverJoystick.getX(), driverJoystick.getThrottle()) * DrivetrainSubsystem.MAX_VELOCITY_MPS,
            () -> -modifyAxis(driverJoystick.getZ(), driverJoystick.getThrottle()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.SwerveDrive.rotateMultiplier
    )
    );
    // m_armSubsystem.setDefaultCommand(new DefaultArmCommand(
    //         m_armSubsystem,
    //         () -> deadband(assistantJoystick.getY(), Constants.UI.deadband) * Constants.Arm.Shoulder.shoulderSpeedMultiplier,
    //         () -> deadband(assistantJoystick.getZ(), Constants.UI.deadband) * Constants.Arm.Elbow.elbowSpeedMultiplier
    // )
    // );
// This line For test purposes  only
       // m_drivetrainSubsystem.setDefaultCommand(new DrivetrainNoAction(m_drivetrainSubsystem));

    // Configure the button bindings
    configureButtonBindings();
    

  }

  // Convert Tracjectory to Ramsete command

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    //new JoystickButton(m_logitech, 3).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
            // No requirements because we don't need to interrupt anything 
    new JoystickButton(driverJoystick, 3).whileTrue(new SetDriveMode(m_drivetrainSubsystem, DriveModes.FieldCentric));
    new JoystickButton(driverJoystick, 4).whileTrue(new SetDriveMode(m_drivetrainSubsystem, DriveModes.RobotCentric));  
    new JoystickButton(driverJoystick, 2).whileTrue(m_ZeroGyroCommand);
    // new JoystickButton(assistantJoystick, 10).whileTrue(m_compressorOff);
    // new JoystickButton(assistantJoystick, 11).whileTrue(m_compressorOn);
    // new JoystickButton(assistantJoystick, 1).whileTrue(m_closegrabber);
    // new JoystickButton(assistantJoystick, 2).whileTrue(m_opengrabber);
    
    // new JoystickButton(assistantJoystick, 3).whileTrue(m_PlatformBalance);
    // new JoystickButton(assistantJoystick, 5).whileTrue(m_ArmParkingCommand);

            // No requirements because we don't need to interrupt anything         
  }

  
  
  
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
    
  }

  public Command getTestCommand() {
    // double angle = 30;//Rotation2d.fromDegrees(30).getDegrees();
    // double speed = 1;
    
    //return new TurnWheelToAngleCommand(m_drivetrainSubsystem, WheelPositions.FrontLeft, speed, angle);
    return null;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }


  //double modifyAxis(double axisValue, double throttleValue){

  

  private static double modifyAxis(double value, double throttleValue) {
    // Deadband 
    double throttle  = 1 -( 0.5 * throttleValue);
    
    value = deadband(value, Constants.UI.deadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return Constants.SwerveDrive.motorSpeed * value * throttle;
  }


  public Command getCompressorStartCommand() {
    // return new TurnCompressorOn(m_grabberSubsystem);
    return new InstantCommand();
  }

//   @Deprecated
//   public Command getInitializeArmTargetRotation() {
//     return new InitializeTargetRotationCommand(m_armSubsystem);
//   }
  
  public void initializeArmSetpoints() {
    // m_armSubsystem.initializeSetpoints();
  }
  public Command getZeroGyroCommand() {
    return new ZeroGyroCommand(m_drivetrainSubsystem);
  }
}
