// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class MoveBackwards extends CommandBase {
  
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private double targetDistance;
  private double startDistance;
  private double speed;
  private Timer timer;
  //private double speed;

  public MoveBackwards(DrivetrainSubsystem driveSystem, double distance, double speed){
  m_drivetrainSubsystem = driveSystem;
  this.targetDistance = distance;
  this.speed = speed;
  this.m_drivetrainSubsystem = driveSystem;
  this.timer = new Timer();
  //targetDistance = target;
    addRequirements(driveSystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
    startDistance = m_drivetrainSubsystem.getDistanceXY(); //- targetDistance;
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
    // if (m_drivetrainSubsystem.getDistanceY() + startDistance < targetDistance){
    //   return false;
    // }else{
    //   m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    //   return true;
    // }

    double distance = Math.abs(m_drivetrainSubsystem.getDistanceXY() - startDistance);
    // if (distance < targetDistance){
    //   return false;
    // }else{
    //   m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    //   return true;}

    if (distance > targetDistance) {
      return true;
    }
    else if (this.timer.get() > targetDistance / speed) {
      return true;
    }
    return false;
  }
}