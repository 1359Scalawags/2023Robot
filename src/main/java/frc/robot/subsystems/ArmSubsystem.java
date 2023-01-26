// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax elbowMotor;
  private CANSparkMax shoulderMotor;
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    elbowMotor = new CANSparkMax(Constants.SwerveDrive.ArmMotors.elbow, MotorType.kBrushless);

    shoulderMotor = new CANSparkMax(Constants.SwerveDrive.ArmMotors.shoulder, MotorType.kBrushless);



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
  public void setelbowMotor(double speed) {
    elbowMotor.set(speed);
  }  
  public void setshoulderMotor(double speed) {
    shoulderMotor.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
