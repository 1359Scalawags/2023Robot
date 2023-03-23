// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

@SuppressWarnings("unused")
public class DelayCommand extends CommandBase {

  private final ArmSubsystem m_subsystem;
  private double delay;
  private Timer timer;

  public DelayCommand(ArmSubsystem subsystem, Double second) {
    this.timer = new Timer();
    delay = second;
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.timer.get() > delay) {
      return true;
    }
    return false;
  }
}
