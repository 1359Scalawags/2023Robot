package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmOnSpecificLevelCommand;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.GrabCommandOpen;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabGamePieceOnGround extends SequentialCommandGroup{
    public GrabGamePieceOnGround(ArmSubsystem m_ArmSubsystem, GrabberSubsystem m_GrabberSubsystem){
        addCommands(
            // new GrabCommandOpen(m_GrabberSubsystem),
            new GrabCommandOpen(m_GrabberSubsystem),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 180.0, 265.0),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 240.0),
            new ArmOnSpecificLevelCommand(m_ArmSubsystem, 133, 265),
            new DelayCommand(m_ArmSubsystem, 0.3),
            new GrabCommandOpen(m_GrabberSubsystem),
            new DelayCommand(m_ArmSubsystem, 0.3),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 200.0, 240.0),
            // new ArmOnSpecificLevelCommand(m_ArmSubsystem, 180.0, 265.0),
            new UnParkingArm(m_ArmSubsystem)
        );
        
    }
}
