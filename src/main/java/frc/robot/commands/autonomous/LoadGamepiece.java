package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmOnHighLevelCommand;
import frc.robot.subsystems.ArmSubsystem;

public class LoadGamepiece extends SequentialCommandGroup{
    public LoadGamepiece(ArmSubsystem subsystem){
        addCommands(
            new ArmOnHighLevelCommand(subsystem)

        );
    }
}
