// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;

import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
public class ArmSubsystem extends SubsystemBase {
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;
  private DutyCycleEncoder elbowEncoder;
  private DutyCycleEncoder shoulderEncoder;
  private SparkMaxPIDController elbowPidController;
  private SparkMaxPIDController shoulderPidController;

  private double elbowTargetSpeed = 0;
  private double shoulderTargetSpeed = 0;

  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    elbowMotor = new SendableCANSparkMax(Constants.Arm.Elbow.motor, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowMotor.setInverted(false);
    elbowMotor.setIdleMode(IdleMode.kBrake);

    shoulderMotor = new SendableCANSparkMax(Constants.Arm.Shoulder.motor, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(false);
    shoulderMotor.setIdleMode(IdleMode.kBrake);

    elbowEncoder = new DutyCycleEncoder(Constants.Arm.Elbow.channel);
    shoulderEncoder = new DutyCycleEncoder(Constants.Arm.Shoulder.channel);
    elbowEncoder.setDistancePerRotation(360.0);
    shoulderEncoder.setDistancePerRotation(360.0);  
    // elbowEncoder.setAverageBits(4); 
    // shoulderEncoder.setAverageBits(4);

    elbowPidController = elbowMotor.getPIDController();
    shoulderPidController = shoulderMotor.getPIDController();
    
    elbowPidController.setP(Constants.Arm.kP);
    elbowPidController.setI(Constants.Arm.kI);
    elbowPidController.setD(Constants.Arm.kD);
    elbowPidController.setIZone(Constants.Arm.kIz);
    elbowPidController.setFF(Constants.Arm.kFF);
    elbowPidController.setOutputRange(Constants.Arm.kMinOutput, Constants.Arm.kMaxOutput);
    
    shoulderPidController.setP(Constants.Arm.kP);
    shoulderPidController.setI(Constants.Arm.kI);
    shoulderPidController.setD(Constants.Arm.kD);
    shoulderPidController.setIZone(Constants.Arm.kIz);
    shoulderPidController.setFF(Constants.Arm.kFF);
    shoulderPidController.setOutputRange(Constants.Arm.kMinOutput, Constants.Arm.kMaxOutput);
    
    addChild("elbowMotor", elbowMotor);
    addChild("shoulderMotor", shoulderMotor);

    tab.add("Shoulder encoder", shoulderEncoder);
    tab.add("Elbow encoder",elbowEncoder);

            // This can either be STANDARD or FAST depending on your gear configuration
            ;
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
    return elbowEncoder.get() < Constants.Arm.Elbow.lowerlimit;
  }
  public boolean isElbowAtLowerLimit() {
    return elbowEncoder.get() > Constants.Arm.Elbow.upperlimit;
  }

  public boolean isShoulderAtUpperLimit() {
    return shoulderEncoder.get() < Constants.Arm.Shoulder.lowerlimit;
  }
  public boolean isshoulderAtLowerLimit() {
    return shoulderEncoder.get() > Constants.Arm.Shoulder.upperlimit;
  }

  public void setElbowMotor(double speed) {
    elbowTargetSpeed = speed;
    //elbowMotor.set(speed);
  }  
  public void setShoulderMotor(double speed) {
    shoulderTargetSpeed = speed;
    //shoulderMotor.set(speed);
  }

  public void setElbowTarget(double angle) {
    angle = Math.min(angle, Constants.Arm.Elbow.upperlimit);
    angle = Math.max(angle, Constants.Arm.Elbow.lowerlimit);
    elbowPidController.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    if(isElbowAtUpperLimit()){
        if(elbowMotor.get() > 0 || elbowTargetSpeed > 0) {
          elbowMotor.stopMotor(); 
          elbowTargetSpeed = 0;          
        }
    } else if (isElbowAtLowerLimit()) {
        if(elbowMotor.get() < 0 || elbowTargetSpeed < 0) {
          elbowMotor.stopMotor(); 
          elbowTargetSpeed = 0; 
        }
    } 

    elbowMotor.set(elbowTargetSpeed);
    
    
    if(isShoulderAtUpperLimit()){
        if (shoulderMotor.get() > 0 || shoulderTargetSpeed > 0) {
            shoulderMotor.stopMotor();
            shoulderTargetSpeed = 0;
        } 
    } else if (isshoulderAtLowerLimit()) {
        if (shoulderMotor.get() < 0 || shoulderTargetSpeed < 0) {
          shoulderMotor.stopMotor(); 
          shoulderTargetSpeed = 0;
      }
  }

    shoulderMotor.set(shoulderTargetSpeed);



    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
