package frc.robot.commands;

// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {
    private ArmSubsystem m_ArmSubsystem;

    private final DoubleSupplier m_Shoulder;
    private final DoubleSupplier m_Elbow;

    // private static int counter = 0;

    // private static ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    // private static GenericEntry counterEntry = tab.add("Counter", counter).getEntry();

    public DefaultArmCommand(ArmSubsystem ArmSubsystem,
                               DoubleSupplier Shoulder,
                               DoubleSupplier Elbow)
 {
        this.m_ArmSubsystem = ArmSubsystem;
        this.m_Elbow = Elbow;
        this.m_Shoulder = Shoulder;

        addRequirements(ArmSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double elbow = m_Elbow.getAsDouble();
        double shoulder = m_Shoulder.getAsDouble();
        // counter++;
        // counterEntry.setInteger(counter);
        // m_ArmSubsystem.changeRelativeSetPoint("shoulder", shoulder);
        // m_ArmSubsystem.changeRelativeSetPoint("elbow", elbow);
        
        // using new functions that have built-in limit checking and setpoint updates
        m_ArmSubsystem.changeElbowSetpoint(elbow);
        m_ArmSubsystem.changeShoulderSetpoint(shoulder);
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return true;

    }
}