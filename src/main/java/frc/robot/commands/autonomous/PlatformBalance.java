// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.SPI;

public class PlatformBalance extends CommandBase {

  private DrivetrainSubsystem m_drivetrainSubsystem;
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
  private static final PIDController pid = new PIDController(Constants.Autonomous.kP, Constants.Autonomous.kI, Constants.Autonomous.kD);


  /** Creates a new ExampleSubsystem. */
  public PlatformBalance(DrivetrainSubsystem driveSystem){
    m_drivetrainSubsystem = driveSystem;
    pid.setSetpoint(0);
    pid.setTolerance(-1, 1);
  }

  @Override
  public void execute() {
    // This method will be called once per scheduler run 
      double K = pid.calculate(m_navx.getPitch());
      m_drivetrainSubsystem.drive(
        new ChassisSpeeds(K,0,0)
      );

  }


}
