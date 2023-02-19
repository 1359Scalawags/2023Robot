// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
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

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;

import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
public class ArmSubsystem extends SubsystemBase {
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;
  private DutyCycleEncoder elbowEncoder;
  private DutyCycleEncoder shoulderEncoder;
  private GenericEntry shoulderPositionEntry;
  private GenericEntry elbowPositionEntry;

  private double elbowMotorSpeed;
  private double shoulderMotorSpeed;
  private ShuffleboardTab tab;
  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    tab = Shuffleboard.getTab("Arm");
    shoulderPositionEntry = tab.add("Shoulder Absolute Position", 0).getEntry();
    elbowPositionEntry = tab.add("Elbow Absolute Position", 0).getEntry();
    elbowMotor = new SendableCANSparkMax(Constants.Arm.Elbow.motor, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowMotor.setInverted(false);
    elbowMotor.setIdleMode(IdleMode.kCoast);

    shoulderMotor = new SendableCANSparkMax(Constants.Arm.Shoulder.motor, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(false);
    shoulderMotor.setIdleMode(IdleMode.kCoast);

    elbowEncoder = new DutyCycleEncoder(Constants.Arm.Elbow.channel);
    shoulderEncoder = new DutyCycleEncoder(Constants.Arm.Shoulder.channel);
    elbowEncoder.setDistancePerRotation(360.0);
    shoulderEncoder.setDistancePerRotation(360.0);  
    // elbowEncoder.setAverageBits(4); 
    // shoulderEncoder.setAverageBits(4);
    
    //tab.add("elbowMotor", elbowMotor);
    //tab.add("shoulderMotor", shoulderMotor);

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
    return elbowEncoder.getAbsolutePosition() * 360 >= Constants.Arm.Elbow.upperlimit;
  }
  public boolean isElbowAtLowerLimit() {
    return elbowEncoder.getAbsolutePosition() * 360 <= Constants.Arm.Elbow.lowerlimit;
  }

  public boolean isShoulderAtUpperLimit() {
    return shoulderEncoder.getAbsolutePosition() * 360 >= Constants.Arm.Shoulder.upperlimit;
  }
  public boolean isshoulderAtLowerLimit() {
    return shoulderEncoder.getAbsolutePosition() * 360 <= Constants.Arm.Shoulder.lowerlimit;
  }

  public void setElbowMotor(double speed) {
    elbowMotorSpeed = speed;
  }  
  public void setShoulderMotor(double speed) {
    shoulderMotorSpeed = speed;
  }
  @Override
  public void periodic() {
    
    shoulderPositionEntry.setDouble(shoulderEncoder.getAbsolutePosition() * 360);
    elbowPositionEntry.setDouble(elbowEncoder.getAbsolutePosition() * 360);

    if(isShoulderAtUpperLimit()){
      if (shoulderMotorSpeed > 0){
          shoulderMotor.stopMotor();
          shoulderMotorSpeed = 0;
      } 
  } 
  if (isshoulderAtLowerLimit()) {
      if (shoulderMotorSpeed < 0) {
        shoulderMotor.stopMotor(); 
        shoulderMotorSpeed = 0;
    }
}
if(shoulderMotorSpeed == 0) {
  //shoulderMotorSpeed = 0.05;
}

  shoulderMotor.set(shoulderMotorSpeed);
  double elbowAdustSpeed = shoulderMotorSpeed * 0.595;
  

    if(isElbowAtUpperLimit()){
      if(elbowMotorSpeed > 0) {
        elbowMotor.stopMotor();    
        elbowMotorSpeed = 0;   
        if(shoulderMotorSpeed < 0)
          elbowAdustSpeed = 0;   
        
      }
  } 

  if (isElbowAtLowerLimit()) {
      if(elbowMotorSpeed < 0) {
        elbowMotor.stopMotor(); 
        elbowMotorSpeed = 0; 
        if(shoulderMotorSpeed > 0)
          elbowAdustSpeed = 0;
        
      }
  } 

  elbowMotor.set(elbowMotorSpeed);
  
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
