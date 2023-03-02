package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetDriveMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveModes;

public class TestAutoMovment extends SequentialCommandGroup
{
    public TestAutoMovment(DrivetrainSubsystem m_DrivetrainSubsystem, boolean includeChargeStation)
    {
        addCommands(
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),

            new MoveAhead(m_DrivetrainSubsystem,1,0.5),

            new MoveBackwards(m_DrivetrainSubsystem,1,0.5),

            new MoveLeft(m_DrivetrainSubsystem, 1, 0.5),

            new MoveRight(m_DrivetrainSubsystem, 1, 0.5));

        if(includeChargeStation) {
            addCommands(null);
        }
    }
}