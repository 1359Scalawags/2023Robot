package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PlatformBalance;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

public class TestMovement extends SequentialCommandGroup
{
    public TestMovement(DrivetrainSubsystem m_DrivetrainSubsystem, boolean includeChargeStation)
    {

        if(includeChargeStation) {
            addCommands(
                // new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
           
                // new GrabCommandClose(m_GrabberSubsystem),
    
                // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),
    
                // new ArmOnHighLevelCommand(m_ArmSubsystem),
    
                // new GrabCommandOpen(m_GrabberSubsystem),
    
                // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),
    
                // new ArmParkingCommand(m_ArmSubsystem),
            
                new MoveBackwards(m_DrivetrainSubsystem,3.5, Constants.Autonomous.autoSpeed),

                new MoveLeft(m_DrivetrainSubsystem, 1.7, Constants.Autonomous.autoSpeed),
    
                new MoveForward(m_DrivetrainSubsystem, 1.55, Constants.Autonomous.autoSpeed),

                new PlatformBalance(m_DrivetrainSubsystem)
            );
        }

        else {
            addCommands(
            // Load cube to the 3rd level grid
            // new GrabCommandOpen(m_GrabberSubsystem),
            // We are moving backwards because the robot will be facing the drivers, not the other side.
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
           
            // new GrabCommandClose(m_GrabberSubsystem),

            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),

            // new ArmOnHighLevelCommand(m_ArmSubsystem),

            // new GrabCommandOpen(m_GrabberSubsystem),

            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),

            // new ArmParkingCommand(m_ArmSubsystem),

            new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed));
        }
    }
}