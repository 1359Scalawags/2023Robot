// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.GrabberSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GrabCommandClose extends CommandBase {

  private GrabberSubsystem m_GrabberSubsystem;

  public GrabCommandClose(GrabberSubsystem driveSystem){
    m_GrabberSubsystem = driveSystem;
    addRequirements(driveSystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() { m_GrabberSubsystem.close();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_GrabberSubsystem.close();
      // if(GrabberSubsystem.isOpen()){
      //   GrabberSubsystem.close();
      // }
      // else{
      //   GrabberSubsystem.open();
      // }
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
