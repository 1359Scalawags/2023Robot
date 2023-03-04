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

public class StandardAuto extends SequentialCommandGroup
{
    public StandardAuto(DrivetrainSubsystem m_DrivetrainSubsystem, ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem, boolean includeChargeStation)
    {
            addCommands(
            // Load cube to the 3rd level grid
            // new GrabCommandOpen(m_GrabberSubsystem),
            // We are moving backwards because the robot will be facing the drivers, not the other side.
            new SetDriveMode(m_DrivetrainSubsystem, DriveModes.RobotCentric),
           
            new GrabCommandClose(m_GrabberSubsystem),

            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),

            new ArmOnHighLevelCommand(m_ArmSubsystem),

            new GrabCommandOpen(m_GrabberSubsystem),

            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 270.0),

            new ArmParkingCommand(m_ArmSubsystem),

            new MoveBackwards(m_DrivetrainSubsystem,3.5,Constants.Autonomous.autoSpeed));
        }
    }