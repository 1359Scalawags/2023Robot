package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElbowUnParkingCommand;
import frc.robot.commands.SetFlagForMovingCommand;
import frc.robot.commands.ShoulderUnParkingCommand;
import frc.robot.subsystems.ArmSubsystem;

public class UnParkingArm extends SequentialCommandGroup{
    public UnParkingArm(ArmSubsystem m_arArmSubsystem) {
        new SequentialCommandGroup(
            new ElbowUnParkingCommand(m_arArmSubsystem),            
            new ShoulderUnParkingCommand(m_arArmSubsystem),
            new SetFlagForMovingCommand(m_arArmSubsystem, true)
        );
    }
    
}
