// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

@SuppressWarnings("unused")
public class EnablePathfining extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    private boolean isPathfindingAuto = true;

    public EnablePathfining(DrivetrainSubsystem driveSystem) {
        this.m_drivetrainSubsystem = driveSystem;
        addRequirements(m_drivetrainSubsystem);
    }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {   
    m_drivetrainSubsystem.setPathfindingMode(true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}