package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmOnHighLevelCommand;
import frc.robot.commands.ArmOnSpecificLevelCommand;
import frc.robot.commands.ArmParkingCommand;
import frc.robot.commands.GrabCommandClose;
import frc.robot.commands.GrabCommandOpen;
import frc.robot.commands.PlatformBalance;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

public class RedStationONE extends SequentialCommandGroup
{
    public RedStationONE(DrivetrainSubsystem m_DrivetrainSubsystem, ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem, boolean includeChargeStation)
    {

        if(includeChargeStation) {
            addCommands(
                new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),

                new GrabCommandClose(m_GrabberSubsystem),
    
                new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),
    
                new ArmOnHighLevelCommand(m_ArmSubsystem),
    
                new GrabCommandOpen(m_GrabberSubsystem),
    
                new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),
    
                new ArmParkingCommand(m_ArmSubsystem),
            
                new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed),

                new MoveRight(m_DrivetrainSubsystem, 1.65, Constants.Autonomous.autoSpeed),
    
                new MoveForward(m_DrivetrainSubsystem, 1.55, Constants.Autonomous.autoSpeed),

                new PlatformBalance(m_DrivetrainSubsystem)
            );
        } else {
            addCommands(
            // Load cube to the 3rd level grid
            // new GrabCommandOpen(m_GrabberSubsystem),
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),

            new GrabCommandClose(m_GrabberSubsystem),

            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),

            new ArmOnHighLevelCommand(m_ArmSubsystem),

            new GrabCommandOpen(m_GrabberSubsystem),

            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),

            new ArmParkingCommand(m_ArmSubsystem),
// We are moving backwards because the robot will be facing the drivers, not the other side.
            new MoveBackwards(m_DrivetrainSubsystem,1,Constants.Autonomous.autoSpeed));

        }
    }
}