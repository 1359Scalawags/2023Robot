package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;
import frc.robot.subsystems.GrabberSubsystem;

public class TestingPathplanner extends SequentialCommandGroup
{
    public TestingPathplanner(DrivetrainSubsystem m_DrivetrainSubsystem, ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem)
    {

        addCommands(

            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),

            new EnablePathfining(m_DrivetrainSubsystem),
           
            // new LoadGamepieceOnHighLevel(m_ArmSubsystem, m_GrabberSubsystem),

            m_DrivetrainSubsystem.followTrajectoryCommand(m_DrivetrainSubsystem.getStartBackwardAndRotate(), true),

            new GrabGamePieceOnGround(m_ArmSubsystem, m_GrabberSubsystem),

            m_DrivetrainSubsystem.followTrajectoryCommand(m_DrivetrainSubsystem.getEndBackwardAndRotate(), true),

            // new LoadGamepieceOnMidLevel(m_ArmSubsystem, m_GrabberSubsystem),

            m_DrivetrainSubsystem.followTrajectoryCommand(m_DrivetrainSubsystem.getStartBackwardAndRotate(), true)


            // m_DrivetrainSubsystem.followTrajectoryCommand(m_DrivetrainSubsystem.getBlue1ToCenter(), true)
            
                // new MoveBackwards(m_DrivetrainSubsystem,3.5, Constants.Autonomous.autoSpeed),

                // new MoveLeft(m_DrivetrainSubsystem, 1.7, Constants.Autonomous.autoSpeed),
    
                // new MoveForward(m_DrivetrainSubsystem, 1.55, Constants.Autonomous.autoSpeed),

                // new PlatformBalance(m_DrivetrainSubsystem)
            );    
    }
}