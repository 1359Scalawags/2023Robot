// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.VisionSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class swichPipeline extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final VisionSystem m_subsystem;
  pipeIndex m_index;
  public enum pipeIndex{
    Default,
    CubeWhiteLight,
    ConeWhiteLight,
    CubeYellowLight,
    ConeYellowLight,
    five,
    six,
    seven
  };

  /** 
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public swichPipeline(VisionSystem subsystem, pipeIndex index ) {
    if (index == null){
      index = pipeIndex.Default;
    }
    m_subsystem = subsystem;
    m_index = index;
  }
  public void set(pipeIndex index){
    m_index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.manualySetPipeline(m_index.ordinal());
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
    return true;
  }
}

