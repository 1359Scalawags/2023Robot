// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;

import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
public class ArmSubsystem extends SubsystemBase {
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;
  private AnalogInput elbowEncoder;
  private AnalogInput shoulderEncoder;
  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    elbowMotor = new SendableCANSparkMax(Constants.SwerveDrive.Arm.elbow.motor, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowMotor.setInverted(false);
    elbowMotor.setIdleMode(IdleMode.kBrake);

    shoulderMotor = new SendableCANSparkMax(Constants.SwerveDrive.Arm.shoulder.motor, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(false);
    shoulderMotor.setIdleMode(IdleMode.kBrake);


    elbowEncoder = new AnalogInput(Constants.SwerveDrive.Arm.elbow.channelA);
    shoulderEncoder = new AnalogInput(Constants.SwerveDrive.Arm.shoulder.channelA);
    elbowEncoder.setAverageBits(4); 
    shoulderEncoder.setAverageBits(4);

    
    addChild("elbowMotor", elbowMotor);
    addChild("shoulderMotor", shoulderMotor);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public CommandBase exampleMethodCommand() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //       });
  // }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }



  public boolean isElbowAtUpperLimit() {
    return elbowEncoder.getAverageValue() < Constants.SwerveDrive.Arm.elbow.lowerlimit;
  }
  public boolean isElbowAtLowerLimit() {
    return elbowEncoder.getAverageValue() > Constants.SwerveDrive.Arm.elbow.upperlimit;
  }

  public boolean isShoulderAtUpperLimit() {
    return shoulderEncoder.getAverageValue() < Constants.SwerveDrive.Arm.shoulder.lowerlimit;
  }
  public boolean isshoulderAtLowerLimit() {
    return shoulderEncoder.getAverageValue() > Constants.SwerveDrive.Arm.shoulder.upperlimit;
  }

  public void setElbowMotor(double speed) {
    elbowMotor.set(speed);
  }  
  public void setShoulderMotor(double speed) {
    shoulderMotor.set(speed);
  }
  @Override
  public void periodic() {
          if(isElbowAtUpperLimit()){
            if (elbowMotor.get() > 0) {
              elbowMotor.stopMotor(); 
            }
          }
          if(isShoulderAtUpperLimit()){
            if (elbowMotor.get() > 0) {
             shoulderMotor.stopMotor();
            } 
          }

          if (isElbowAtLowerLimit()) {
            if (elbowMotor.get() < 0) {
             elbowMotor.stopMotor(); 
            }
          }

          if (isshoulderAtLowerLimit()) { 
            if (elbowMotor.get() < 0) {
              shoulderMotor.stopMotor();
            }
          }

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
