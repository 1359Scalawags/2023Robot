package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PlatformBalance;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

//TODO: figure out what to do for the middle starting positon

public class RedStationTWO extends SequentialCommandGroup
{
    public RedStationTWO(DrivetrainSubsystem m_DrivetrainSubsystem, boolean includeChargeStation)
    {
        addCommands(
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
// We are moving backwards because the robot will be facing the drivers, not the other side.
            new MoveBackwards(m_DrivetrainSubsystem,1,Constants.Autonomous.autoSpeed));

        if(includeChargeStation) {
            addCommands(
                new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
            
                new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed),

                new MoveLeft(m_DrivetrainSubsystem, 1.65, Constants.Autonomous.autoSpeed),
    
                new MoveForward(m_DrivetrainSubsystem, 1.55, Constants.Autonomous.autoSpeed),

                new PlatformBalance(m_DrivetrainSubsystem)
            );
        }
    }
}