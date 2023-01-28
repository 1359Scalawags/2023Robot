// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
public class ArmSubsystem extends SubsystemBase {
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;
  private Encoder elbowEncoder;
  private Encoder shoulderEncoder;
  
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


    elbowEncoder = new Encoder(Constants.SwerveDrive.Arm.elbow.channelA, Constants.SwerveDrive.Arm.elbow.channelB, false, Encoder.EncodingType.k2X);
    shoulderEncoder = new Encoder(Constants.SwerveDrive.Arm.shoulder.channelA, Constants.SwerveDrive.Arm.shoulder.channelB, false, Encoder.EncodingType.k2X);


    
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

  public void rotateUpperArm() {

  }

  public boolean isArmAtUpperLimit() {
    //TODO: Fill this out when know more about the upper arm
    return false;
  }

  public void rotateLowerArm() {

  } 

  public boolean isArmAtLowerLimit() {
    //TODO: Fill this out when know more about the lower arm
    return false;
  }
  public void setElbowMotor(double speed) {
    elbowMotor.set(speed);
  }  
  public void setShoulderMotor(double speed) {
    shoulderMotor.set(speed);
  }
  @Override
  public void periodic() {
          if (elbowEncoder.get() > Constants.SwerveDrive.Arm.elbow.lowerlimit) {
            if (elbowMotor.getAppliedOutput() > 0) {
              elbowMotor.stopMotor(); 
            }
          }
          if (shoulderEncoder.get() > Constants.SwerveDrive.Arm.shoulder.lowerlimit) { 
            if (elbowMotor.getAppliedOutput() > 0) {
             shoulderMotor.stopMotor();
            } 
          }
          if (elbowEncoder.get() < Constants.SwerveDrive.Arm.elbow.upperlimit) {
            if (elbowMotor.getAppliedOutput() < 0) {
             elbowMotor.stopMotor(); 
            }
          }
          if (shoulderEncoder.get() < Constants.SwerveDrive.Arm.shoulder.upperlimit) { 
            if (elbowMotor.getAppliedOutput() < 0) {
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
