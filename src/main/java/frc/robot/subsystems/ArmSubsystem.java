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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  
  GenericEntry shoulderRotationEntry;
  GenericEntry elbowRotationEntry;

  private double elbowTargetSpeed = 0;
  private double shoulderTargetSpeed = 0;

  private double e_kP = Constants.Arm.Elbow.kP * Constants.Arm.Elbow.CoefficientMultiplier,
                 e_kI = Constants.Arm.Elbow.kI * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_kD = Constants.Arm.Elbow.kD * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_kIz = Constants.Arm.Elbow.kIz * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_kFF = Constants.Arm.Elbow.kFF * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_kMaxOutput = Constants.Arm.Elbow.kMaxOutput * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_kMinOutput = Constants.Arm.Elbow.kMinOutput * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_TargetRotation;
  private double s_kP = Constants.Arm.Shoulder.kP * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kI = Constants.Arm.Shoulder.kI * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kD = Constants.Arm.Shoulder.kD * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kIz = Constants.Arm.Shoulder.kIz * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kFF = Constants.Arm.Shoulder.kFF * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kMaxOutput = Constants.Arm.Shoulder.kMaxOutput * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kMinOutput = Constants.Arm.Shoulder.kMinOutput * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_TargetRotation;


  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
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
    
    elbowPidController.setP(e_kP);
    elbowPidController.setI(e_kI);
    elbowPidController.setD(e_kD);
    elbowPidController.setIZone(e_kIz);
    elbowPidController.setFF(e_kFF);
    elbowPidController.setOutputRange(e_kMinOutput, e_kMaxOutput);
    
    shoulderPidController.setP(s_kP);
    shoulderPidController.setI(s_kI);
    shoulderPidController.setD(s_kD);
    shoulderPidController.setIZone(s_kIz);
    shoulderPidController.setFF(s_kFF);
    shoulderPidController.setOutputRange(s_kMinOutput, s_kMaxOutput);
    
    addChild("elbowMotor", elbowMotor);
    addChild("shoulderMotor", shoulderMotor);

    shoulderRotationEntry = tab.add("Shoulder rotation", 0).getEntry();
    elbowRotationEntry = tab.add("Elbow rotation", 0).getEntry();
    tab.add("Shoulder encoder", shoulderEncoder);
    tab.add("Elbow encoder",elbowEncoder);

    tab.add("Shoulder P", shoulderPidController.getP());
    tab.add("Shoulder I", shoulderPidController.getI()); 
    tab.add("Shoulder D", shoulderPidController.getD());
    //tab.add("Shoulder Iz", shoulderPidController.getIZone()); 
    tab.add("Shoulder FF", shoulderPidController.getFF());
    //tab.add("Shoulder Output Max", shoulderPidController.getOutputMax());
    //tab.add("Shoulder Output Min", shoulderPidController.getOutputMin());
   
    tab.add("elbow P", elbowPidController.getP());
    tab.add("elbow I", elbowPidController.getI()); 
    tab.add("elbow D", elbowPidController.getD());
    //tab.add("elbow Iz", elbowPidController.getIZone()); 
    tab.add("elbow FF", elbowPidController.getFF());
    //tab.add("elbow Output Max", elbowPidController.getOutputMax());
    //tab.add("elbow Output Min", elbowPidController.getOutputMin());
    

    shoulderPidController.setReference(shoulderEncoder.getAbsolutePosition(), CANSparkMax.ControlType.kPosition);
    elbowPidController.setReference(elbowEncoder.getAbsolutePosition(), CANSparkMax.ControlType.kPosition);  


            // This can either be STANDARD or FAST depending on your gear configuration
            
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

  public void setShoulderTarget(double angle) {
    angle = Math.min(angle, Constants.Arm.Shoulder.upperlimit);
    angle = Math.max(angle, Constants.Arm.Shoulder.lowerlimit);
    shoulderPidController.setReference(angle, ControlType.kPosition);
  }

  public void changeElbowTarget(double delta) {
    double angle = elbowEncoder.getAbsolutePosition();
    setElbowTarget(angle);
  }

  public void changeShoulderTarget(double delta) {
    double angle = shoulderEncoder.getAbsolutePosition();
    setShoulderTarget(angle);
  }

  @Override
  public void periodic() {

    double e_p = elbowPidController.getP();
    double e_i = elbowPidController.getI();
    double e_d = elbowPidController.getD();
    //double e_Iz = elbowPidController.getIZone();
    double e_FF = elbowPidController.getFF();
    //double e_max = elbowPidController.getOutputMax();
    //double e_min = elbowPidController.getOutputMin();
    //double e_CurrentRotation = elbowEncoder.getAbsolutePosition();

    double s_p = shoulderPidController.getP();
    double s_i = shoulderPidController.getI();
    double s_d = shoulderPidController.getD();
    //double s_Iz = shoulderPidController.getIZone();
    double s_FF = shoulderPidController.getFF();
    //double s_max = shoulderPidController.getOutputMax();
    //double s_min = shoulderPidController.getOutputMin();
    //double s_CurrentRotation = shoulderEncoder.getAbsolutePosition();

    if (e_p != e_kP) {elbowPidController.setP(e_p); e_kP = e_p;}
    if (e_i != e_kI) {elbowPidController.setP(e_i); e_kI = e_i;}
    if (e_d != e_kD) {elbowPidController.setP(e_d); e_kD = e_d;}
    //if (e_Iz != e_kIz) {elbowPidController.setP(e_Iz); e_kIz = e_Iz;}
    if (e_FF != e_kFF) {elbowPidController.setP(e_FF); e_kFF = e_FF;}
    //if (e_max != e_kMaxOutput || e_min != e_kMinOutput) {
      //elbowPidController.setOutputRange(e_min, e_max); 
      //e_kMaxOutput = e_max;
      //e_kMinOutput = e_min;
    //}
    
    if (s_p != s_kP) {elbowPidController.setP(s_p); s_kP = s_p;}
    if (s_i != s_kI) {elbowPidController.setP(s_i); s_kI = s_i;}
    if (s_d != s_kD) {elbowPidController.setP(s_d); s_kD = s_d;}
    //if (s_Iz != s_kIz) {elbowPidController.setP(s_Iz); s_kIz = s_Iz;}
    if (s_FF != s_kFF) {elbowPidController.setP(s_FF); s_kFF = s_FF;}
    //if (s_max != s_kMaxOutput || s_min != s_kMinOutput) {
    //  elbowPidController.setOutputRange(s_min, s_max); 
    //  s_kMaxOutput = s_max;
    //  s_kMinOutput = s_min;
    //}

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


