// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
/** An example command that uses an example subsystem. */


public class MoveLeft extends CommandBase {

  private DrivetrainSubsystem m_drivetrainSubsystem;
  private double startDistance;
  private double targetDistance;
  private double speed;

  public MoveLeft(DrivetrainSubsystem driveSystem, double distance, double speed){
    this.startDistance = distance;
    this.speed = speed;
    m_drivetrainSubsystem = driveSystem;
    addRequirements(driveSystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    startDistance = m_drivetrainSubsystem.getDistanceX(); //+ endDistance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(
    new ChassisSpeeds(-speed,0,0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distance = Math.abs(m_drivetrainSubsystem.getDistanceX() - startDistance);
    if (distance < targetDistance){
      return false;
    }else{
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
      return true;
    }
  }
}