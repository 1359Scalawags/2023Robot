// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.SPI;

public class PlatformBalance extends SubsystemBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
  /** Creates a new ExampleSubsystem. */
  public PlatformBalance(DrivetrainSubsystem driveSystem){
    m_drivetrainSubsystem = driveSystem;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    if(m_navx.getAngle() > 10){
      m_drivetrainSubsystem.drive(
        new ChassisSpeeds(1,0,0)
      );
    }
    else if (m_navx.getAngle() < 10){
     m_drivetrainSubsystem.drive(
        new ChassisSpeeds(-1,0,0)
      );
   }else{
    m_drivetrainSubsystem.drive(
      new ChassisSpeeds(0,0,0)
    );
   }
    // This method will be called once per scheduler run during simulation
  }
}
