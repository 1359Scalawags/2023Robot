package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;
import frc.robot.subsystems.GrabberSubsystem;

public class OnlyMoving extends SequentialCommandGroup
{
    public OnlyMoving(DrivetrainSubsystem m_DrivetrainSubsystem)
    {
            addCommands(
            // Load cube to the 3rd level grid
            // new GrabCommandOpen(m_GrabberSubsystem),
            // We are moving backwards because the robot will be facing the drivers, not the other side.
            new DisablePathfining(m_DrivetrainSubsystem),

            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
           
            // new LoadGamepieceOnHighLevel(m_ArmSubsystem, m_GrabberSubsystem),
            
            new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed)
            );
        }
    }
