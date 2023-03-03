// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmOnSpecificLevelCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ArmSubsystem m_subsystem;
  private double elbowTargetDegree;
  private double shoulderTargetDegree;
  private final SlewRateLimiter e_Limiter;
  private final SlewRateLimiter s_Limiter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmOnSpecificLevelCommand(ArmSubsystem subsystem, double elbowTargetDegree, double shoulderTargetDegree) {
    m_subsystem = subsystem;
    e_Limiter = new SlewRateLimiter(Constants.Arm.Elbow.slewRateLimiter);
    s_Limiter = new SlewRateLimiter(Constants.Arm.Shoulder.slewRateLimiter);
    this.elbowTargetDegree = elbowTargetDegree;
    this.shoulderTargetDegree = shoulderTargetDegree;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    e_Limiter.calculate(m_subsystem.getElbowDegree());
    s_Limiter.calculate(m_subsystem.getShoulderDegree());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setElbowSetpoint(e_Limiter.calculate(Constants.Arm.Elbow.parkingDegree));
    m_subsystem.setShoulderSetpoint(s_Limiter.calculate(Constants.Arm.Shoulder.parkingDegree));
    //TODO: What is optimal angles for parking?
    m_subsystem.setElbowSetpoint(elbowTargetDegree);
    m_subsystem.setShoulderSetpoint(shoulderTargetDegree);
  }

  // Called once the command ends sor is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isElbowAtTarget(Constants.Arm.Elbow.tolerance) && m_subsystem.isShoulderAtTarget(Constants.Arm.Shoulder.tolerance);
  }
}
