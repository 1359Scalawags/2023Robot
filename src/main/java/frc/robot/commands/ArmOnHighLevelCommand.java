// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmOnHighLevelCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ArmSubsystem m_subsystem;
  // private final SlewRateLimiter e_Limiter;
  // private final SlewRateLimiter s_Limiter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmOnHighLevelCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // e_Limiter = new SlewRateLimiter(Constants.Arm.Elbow.slewRateLimiter);
    // s_Limiter = new SlewRateLimiter(Constants.Arm.Shoulder.slewRateLimiter);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // e_Limiter.reset(m_subsystem.getElbowDegree());
    // s_Limiter.reset(m_subsystem.getShoulderDegree());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: What is optimal angles for parking?
    // m_subsystem.setElbowSetpoint(e_Limiter.calculate(Constants.Arm.Elbow.onHighLevel));
    // m_subsystem.setShoulderSetpoint(s_Limiter.calculate(Constants.Arm.Shoulder.onHighLevel));
    m_subsystem.setElbowSetpoint(Constants.Arm.Elbow.onHighLevel);
    m_subsystem.setShoulderSetpoint(Constants.Arm.Shoulder.onHighLevel);
    
    // m_subsystem.setElbowSetpoint(Constants.Arm.Elbow.onHighLevel);
    // m_subsystem.setShoulderSetpoint(Constants.Arm.Shoulder.onHighLevel);
  }
  

  // Called once the command ends sor is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isElbowAtTarget(Constants.Arm.Elbow.onHighLevel, Constants.Arm.Elbow.tolerance) && m_subsystem.isShoulderAtTarget(Constants.Arm.Shoulder.onHighLevel, Constants.Arm.Shoulder.tolerance);
  }
}
