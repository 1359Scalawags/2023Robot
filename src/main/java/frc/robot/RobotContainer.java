// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmOnSubStationCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GrabCommandClose;
import frc.robot.commands.GrabCommandOpen;
import frc.robot.commands.InitializeArm;
import frc.robot.commands.PlatformBalance;
import frc.robot.commands.SetDriveMode;
import frc.robot.commands.TurnCompressorOn;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.commands.SwitchPipeline;
import frc.robot.commands.autonomous.LoadGamepieceOnGroundLevel;
import frc.robot.commands.autonomous.LoadGamepieceOnHighLevel;
import frc.robot.commands.autonomous.LoadGamepieceOnMidLevel;
import frc.robot.commands.autonomous.ParkingArm;
import frc.robot.commands.autonomous.RotateToGamepiece;
import frc.robot.commands.autonomous.UnParkingArm;
import frc.robot.commands.SwitchPipeline.pipeIndex;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DisplaySubSystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PlatformBalance m_PlatformBalance = new PlatformBalance(m_drivetrainSubsystem);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
  private final ZeroGyroCommand m_ZeroGyroCommand = new ZeroGyroCommand(m_drivetrainSubsystem);
  // private final TurnCompressorOff m_compressorOff = new TurnCompressorOff(m_grabberSubsystem);
  // private final TurnCompressorOn m_compressorOn = new TurnCompressorOn(m_grabberSubsystem);
  private final GrabCommandOpen m_opengrabber = new GrabCommandOpen(m_grabberSubsystem);
  private final GrabCommandClose m_closegrabber = new GrabCommandClose(m_grabberSubsystem); 
  private final ParkingArm m_ArmParkingCommand = new ParkingArm(m_armSubsystem);
  private final UnParkingArm m_ArmUnParkingCommand = new UnParkingArm(m_armSubsystem);
  private final LoadGamepieceOnHighLevel m_GamepieceOnHighLevel = new LoadGamepieceOnHighLevel(m_armSubsystem, m_grabberSubsystem);
  private final LoadGamepieceOnMidLevel m_GamepieceOnMidLevel = new LoadGamepieceOnMidLevel(m_armSubsystem, m_grabberSubsystem);
  private final LoadGamepieceOnGroundLevel m_GamepieceOnGroundLevel = new LoadGamepieceOnGroundLevel(m_armSubsystem, m_grabberSubsystem);
  // private final ArmOnGroundLevelCommand m_ArmOnGroundLevelCommand = new ArmOnGroundLevelCommand(m_armSubsystem);
  // private final ArmOnMidLevelCommand m_ArmOnMidLevelCommand = new ArmOnMidLevelCommand(m_armSubsystem);
  // private final ArmOnHighLevelCommand m_ArmOnHighLevelCommand = new ArmOnHighLevelCommand(m_armSubsystem);
  private final ArmOnSubStationCommand m_ArmOnSubStationCommand = new ArmOnSubStationCommand(m_armSubsystem);
  // private final VisionSystem m_VisionSystem = new VisionSystem();
  private final VisionSystem m_VisionSystem = new VisionSystem();
  private final SwitchPipeline detectCone = new SwitchPipeline(m_VisionSystem, pipeIndex.ConeWhiteLight);
  private final SwitchPipeline detectCube = new SwitchPipeline(m_VisionSystem, pipeIndex.CubeWhiteLight);
  private final SetDriveMode fieldCentric = new SetDriveMode(m_drivetrainSubsystem, DriveModes.FieldCentric);
  private final SetDriveMode robotCentric = new SetDriveMode(m_drivetrainSubsystem, DriveModes.RobotCentric);
  private final RotateToGamepiece rotateToGamepiece = new RotateToGamepiece(m_VisionSystem, m_drivetrainSubsystem, null);

  private final DisplaySubSystem m_DisplaySystem = new DisplaySubSystem(m_VisionSystem, m_drivetrainSubsystem, m_armSubsystem, m_grabberSubsystem);

  
  //private final XboxController m_controller = new XboxController(0);
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick assistantJoystick = new Joystick(1);


  public boolean displayAdvancedDashboard = false;
  private final SendableChooser<Boolean> displayAdvanced =  new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_grabberSubsystem.TurnOn(); 


    displayAdvanced.setDefaultOption("See somethings", false);
    displayAdvanced.addOption("See everything", true);
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
    m_armSubsystem.setDefaultCommand(new DefaultArmCommand(
            m_armSubsystem,
            () -> deadband(assistantJoystick.getY(), Constants.UI.deadband) * Constants.Arm.Shoulder.shoulderSpeedMultiplier,
            () -> deadband(assistantJoystick.getZ(), Constants.UI.deadband) * Constants.Arm.Elbow.elbowSpeedMultiplier
    )
    );

    // SendableChooser<Command> chooser = new SendableChooser<>();
    // Shuffleboard.getTab("Autonomous").add(chooser);
    //chooser.addOption(null, new SequentialCommand(m_SequentialCommand, true));


// This line For test purposes  only
       // m_drivetrainSubsystem.setDefaultCommand(new DrivetrainNoAction(m_drivetrainSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    //new JoystickButton(m_logitech, 3).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    new JoystickButton(driverJoystick, 2).whileTrue(m_ZeroGyroCommand);
    new JoystickButton(driverJoystick, 3).whileTrue(fieldCentric);
    new JoystickButton(driverJoystick,4).whileTrue(robotCentric); 
    new JoystickButton(driverJoystick, 5).whileTrue(rotateToGamepiece);
    new JoystickButton(driverJoystick, 6).onTrue(detectCube);
    new JoystickButton(driverJoystick, 7).onTrue(detectCone);
    // new JoystickButton(assistantJoystick, 10).whileTrue(m_compressorOff);
    // new JoystickButton(assistantJoystick, 11).whileTrue(m_compressorOn);
    new JoystickButton(assistantJoystick, 1).whileTrue(m_closegrabber);
    new JoystickButton(assistantJoystick, 2).whileTrue(m_opengrabber);
    // new JoystickButton(assistantJoystick, 3).whileTrue(detectCube);
    // new JoystickButton(assistantJoystick, 4).whileTrue(detectCone);
    new JoystickButton(assistantJoystick, 5).whileTrue(m_ArmParkingCommand);
    new JoystickButton(assistantJoystick, 6).whileTrue(m_ArmUnParkingCommand);
    new JoystickButton(assistantJoystick, 7).whileTrue(m_ArmOnSubStationCommand);
    // new JoystickButton(assistantJoystick, 7).onTrue(m_GamepieceOnGroundLevel);
    new JoystickButton(assistantJoystick, 8).whileTrue(m_GamepieceOnHighLevel);
    new JoystickButton(assistantJoystick, 9).whileTrue(m_GamepieceOnMidLevel);
    new JoystickButton(assistantJoystick, 10).whileTrue(m_GamepieceOnGroundLevel);
    // new JoystickButton(assistantJoystick, 6).onTrue(m_ArmOnGroundLevelCommand);
    // new JoystickButton(assistantJoystick, 7).onTrue(m_ArmOnMidLevelCommand);
    // new JoystickButton(assistantJoystick, 8).onTrue(m_ArmOnHighLevelCommand);
    // new JoystickButton(assistantJoystick, 9).onTrue(m_ArmOnSubStationCommand);
    

            // No requirements because we don't need to interrupt anything         
  }
  
  
  
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_DisplaySystem.getAutonomousChooser().getSelected();
    
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
    return new TurnCompressorOn(m_grabberSubsystem);
  }

//   @Deprecated
//   public Command getInitializeArmTargetRotation() {
//     return new InitializeTargetRotationCommand(m_armSubsystem);
//   }
  
  // public void initializeArmSetpoints() {
  //   m_armSubsystem.initializeSetpoints();
  // }

  public Command getInitializeArmCommand() {
    return new InitializeArm(m_armSubsystem);
  }

  public Command zeroGyro() {
    return new ZeroGyroCommand(m_drivetrainSubsystem);
  }


  public Command grabPiece() {
    // System.out.println("Close Grabber");
    return new GrabCommandClose(m_grabberSubsystem);
  }

  // public void setPinAutonomous(){
  //   m_armSubsystem.setPinAutonomous();
  // }

  public void setPinTeleop(){
    
  }
  // public void openGrabber() {
  //   m_grabberSubsystem.open();
  // }
}
