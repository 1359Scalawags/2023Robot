// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.extensions.Utilities;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSystem;
/** An example command that uses an example subsystem. */
public class MoveForwardToTarget extends CommandBase {


  private DrivetrainSubsystem m_drivetrainSubsystem;
  private VisionSystem m_VisionSystem;
  private double targetDistance;
  //private double startDistance;
  private double speed;
  private Timer timer;
  
  public MoveForwardToTarget(DrivetrainSubsystem driveSystem, VisionSystem visionSystem, double speed){
    m_drivetrainSubsystem = driveSystem;
    m_VisionSystem = visionSystem;
    this.targetDistance = visionSystem.getDistanceFromTarget();
    this.speed = speed;
    this.m_drivetrainSubsystem = driveSystem;
    this.timer = new Timer();
    addRequirements(driveSystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    this.timer.reset();
    this.timer.start();
    // startDistance = m_drivetrainSubsystem.getDistanceFwdBwd(); //- targetDistance;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(
    new ChassisSpeeds(speed,0,0)
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
    // double distance = Math.abs(m_drivetrainSubsystem.getDistanceFwdBwd() - startDistance);
    // if (distance < targetDistance){
    //   return false;
    // }else{
    //   m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    //   return true;
    // }
    if (this.timer.get() > targetDistance / speed || Utilities.IsCloseTo(m_VisionSystem.getDistanceFromTarget(), targetDistance, 0.5)) {
      return true;
    }
    return false;
  }
}
