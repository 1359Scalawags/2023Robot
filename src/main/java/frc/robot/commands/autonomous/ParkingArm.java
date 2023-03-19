package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElbowParkingCommand;
import frc.robot.commands.SetFlagForMovingCommand;
import frc.robot.commands.ShoulderParkingCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ParkingArm extends SequentialCommandGroup{
    public ParkingArm(ArmSubsystem m_arArmSubsystem) {
        addCommands(
            new SetFlagForMovingCommand(m_arArmSubsystem, false),
            new ShoulderParkingCommand(m_arArmSubsystem),
            new ElbowParkingCommand(m_arArmSubsystem)
        );
    }
    
}
