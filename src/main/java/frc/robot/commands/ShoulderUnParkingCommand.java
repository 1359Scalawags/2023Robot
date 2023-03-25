// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ShoulderUnParkingCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ArmSubsystem m_subsystem;
  private final Timer timer;
  // private final SlewRateLimiter e_Limiter;
  // private final SlewRateLimiter s_Limiter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShoulderUnParkingCommand(ArmSubsystem subsystem) {
    this.timer = new Timer();
    m_subsystem = subsystem;
    // e_Limiter = new SlewRateLimiter(Constants.Arm.Elbow.slewRateLimiter);
    // s_Limiter = new SlewRateLimiter(Constants.Arm.Shoulder.slewRateLimiter);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
    System.out.println("Init : Shoulderunpark");
    // e_Limiter.reset(m_subsystem.getElbowDegree());
    // s_Limiter.reset(m_subsystem.getShoulderDegree());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.setElbowSetpoint(e_Limiter.calculate(Constants.Arm.Elbow.parkingDegree));
    // m_subsystem.setShoulderSetpoint(s_Limiter.calculate(Constants.Arm.Shoulder.parkingDegree));
    m_subsystem.unParkShoulder();
    // m_subsystem.setElbowSetpoint(Constants.Arm.Elbow.parkingDegree);
    // m_subsystem.setShoulderSetpoint(Constants.Arm.Shoulder.parkingDegree);
  }

  // Called once the command ends sor is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End : Shoulderunpark");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.timer.get() > 1.5) {
      return true;
    }
    return m_subsystem.isShoulderAtTarget(Constants.Arm.Shoulder.unParkingDegree, Constants.Arm.parkingTolerance);
  }
}
