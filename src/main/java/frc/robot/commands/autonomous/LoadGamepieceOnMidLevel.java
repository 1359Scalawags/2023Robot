package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmOnMidLevelCommand;
import frc.robot.commands.ArmOnSpecificLevelCommand;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.GrabCommandClose;
import frc.robot.commands.GrabCommandOpen;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class LoadGamepieceOnMidLevel extends SequentialCommandGroup{
    public LoadGamepieceOnMidLevel(ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem){
        addCommands(
            // new GrabCommandOpen(m_GrabberSubsystem),
            new GrabCommandClose(m_GrabberSubsystem),
            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 203.0, 269.0),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 240.0),
            new ArmOnMidLevelCommand(m_ArmSubsystem),
            new DelayCommand(m_ArmSubsystem, 0.3),
            new GrabCommandOpen(m_GrabberSubsystem),
            new DelayCommand(m_ArmSubsystem, 0.3),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 240.0),
            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 203.0, 269.0),
            new UnParkingArm(m_ArmSubsystem)
        );
    }
}
