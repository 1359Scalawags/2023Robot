package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmOnHighLevelCommand;
import frc.robot.commands.ArmOnSpecificLevelCommand;
import frc.robot.commands.ArmParkingCommand;
import frc.robot.commands.GrabCommandClose;
import frc.robot.commands.GrabCommandOpen;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class LoadGamepiece extends SequentialCommandGroup{
    public LoadGamepiece(ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem){
        addCommands(
            // new GrabCommandOpen(m_GrabberSubsystem),
            new GrabCommandClose(m_GrabberSubsystem),
            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 260.0),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 240.0),
            new ArmOnHighLevelCommand(m_ArmSubsystem),
            new GrabCommandOpen(m_GrabberSubsystem),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 240.0),
            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 260.0),
            new ArmParkingCommand(m_ArmSubsystem)
        );
    }
}
