// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.VisionSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ChoosePipeLineByLightAndTarget extends CommandBase {

  private final VisionSystem m_subsystem;
  private final Lighting m_lighting;
  private final Targets m_target;

  public enum Lighting {
    White,
    Yellow
  }

  public enum Targets {
    Cone,
    Cube,
    Pole
  }





  /** 
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChoosePipeLineByLightAndTarget(VisionSystem subsystem, Lighting light, Targets target) {
    m_subsystem = subsystem;
    m_lighting = light;
    m_target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_lighting.ordinal() == 0){
       
      if (m_target.ordinal() == 0){
        m_subsystem.manualySetPipeline(1);
      }else if (m_target.ordinal() == 1){
        m_subsystem.manualySetPipeline(2);
      }else{
        m_subsystem.manualySetPipeline(5);
      }

    }else{

      if (m_target.ordinal() == 0){
        m_subsystem.manualySetPipeline(3);
      }else if (m_target.ordinal() == 1){ 
        m_subsystem.manualySetPipeline(4);
      }else{
        m_subsystem.manualySetPipeline(5);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

