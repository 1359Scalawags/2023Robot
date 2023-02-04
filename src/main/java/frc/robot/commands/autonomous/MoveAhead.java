// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;
import javax.swing.DefaultDesktopManager;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
/** An example command that uses an example subsystem. */
public class MoveAhead extends CommandBase {


  private DrivetrainSubsystem m_drivetrainSubsystem;

  public MoveAhead(DrivetrainSubsystem driveSystem){
    m_drivetrainSubsystem = driveSystem;
    addRequirements(driveSystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(
    new ChassisSpeeds(1,0,0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
