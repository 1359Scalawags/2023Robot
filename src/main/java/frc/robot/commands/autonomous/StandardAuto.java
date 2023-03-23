package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetDriveMode;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;
import frc.robot.subsystems.GrabberSubsystem;

public class StandardAuto extends SequentialCommandGroup
{
    public StandardAuto(DrivetrainSubsystem m_DrivetrainSubsystem, ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem, boolean includeChargeStation)
    {
            addCommands(
            // Load cube to the 3rd level grid
            // new GrabCommandOpen(m_GrabberSubsystem),
            // We are moving backwards because the robot will be facing the drivers, not the other side.
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
           
            new LoadGamepieceOnHighLevel(m_ArmSubsystem, m_GrabberSubsystem),
            
            new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed),

            new TurnToAngle(180),

            new GrabGamePieceOnGround(m_ArmSubsystem, m_GrabberSubsystem),

            new TurnToAngle(180),

            new MoveForward(m_DrivetrainSubsystem, 3.5, Constants.Autonomous.autoSpeed),

            new LoadGamepieceOnMidLevel(m_ArmSubsystem, m_GrabberSubsystem)
            );
        }
    }
