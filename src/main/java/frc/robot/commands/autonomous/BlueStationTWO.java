package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

//TODO: figure out what to do for the middle starting position.

public class BlueStationTWO extends SequentialCommandGroup
{
    public BlueStationTWO(DrivetrainSubsystem m_DrivetrainSubsystem, ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem, boolean includeChargeStation)
    {


        if(includeChargeStation) {
            addCommands(
                new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),

                new LoadGamepieceOnHighLevel(m_ArmSubsystem, m_GrabberSubsystem)
            
                // new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed),

                // new MoveLeft(m_DrivetrainSubsystem, 1.65, Constants.Autonomous.autoSpeed),
    
                // new MoveForward(m_DrivetrainSubsystem, 1.55, Constants.Autonomous.autoSpeed),

                // new PlatformBalance(m_DrivetrainSubsystem)
            );
        }

        else {
            addCommands(
            // Load cube to the 3rd level grid
            // new GrabCommandOpen(m_GrabberSubsystem),
            // We are moving backwards because the robot will be facing the drivers, not the other side.
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),

            new LoadGamepieceOnHighLevel(m_ArmSubsystem, m_GrabberSubsystem)
            
            //new MoveBackwards(m_DrivetrainSubsystem,3.5, Constants.Autonomous.autoSpeed)
            );
        }
    }
}