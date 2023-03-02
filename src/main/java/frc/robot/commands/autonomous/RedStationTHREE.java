package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

//TODO: messure feild and put in distances.

public class RedStationTHREE extends SequentialCommandGroup
{
    public RedStationTHREE(DrivetrainSubsystem m_DrivetrainSubsystem, boolean includeChargeStation)
    {
        addCommands(
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
// We are moving backwards because the robot will be facing the drivers, not the other side.
            new MoveBackwards(m_DrivetrainSubsystem,1,0.5));

        if(includeChargeStation) {
            addCommands(
                new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
            
                new MoveBackwards(m_DrivetrainSubsystem,1,0.5),

                new MoveLeft(m_DrivetrainSubsystem, 1, 0.5),
    
                new MoveForward(m_DrivetrainSubsystem, 1, 0.5)
            );
        }
    }
}