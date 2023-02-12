package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {
    private ArmSubsystem m_ArmSubsystem;

    private final DoubleSupplier m_Shoulder;
    private final DoubleSupplier m_Elbow;

    private static int counter = 2;

    private static ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    private static GenericEntry counterEntry = tab.add("Counter", counter).getEntry();

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
    public void execute() {
        counter++;
        counterEntry.setInteger(counter);
        m_ArmSubsystem.changeShoulderTarget(m_Shoulder.getAsDouble());
        m_ArmSubsystem.changeElbowTarget(m_Elbow.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
       return;
    }

    public boolean isFinished() {
        return true;

    }
}