// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.extensions.FloorRelativeEncoder;
import frc.robot.extensions.SendableCANSparkMax;

public class ArmSubsystem extends SubsystemBase {
  
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;
  //private DutyCycleEncoder elbowEncoder;
  //private DutyCycleEncoder shoulderEncoder;
  private FloorRelativeEncoder elbowRelativeEncoder;
  private FloorRelativeEncoder shoulderRelativeEncoder;

  private PIDController elbowPidController;
  private PIDController shoulderPidController;
  private ArmFeedforward elbowFFController;
  private ArmFeedforward shoulderFFController;
  
  private GenericEntry shoulderRotationEntry;
  private GenericEntry elbowRotationEntry;
  

  ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  // private double elbowTargetSpeed = 0;
  // private double shoulderTargetSpeed = 0;

  private double e_kP = Constants.Arm.Elbow.kP * Constants.Arm.Elbow.CoefficientMultiplier,
                 e_kI = Constants.Arm.Elbow.kI * Constants.Arm.Elbow.CoefficientMultiplier, 
                 e_kD = Constants.Arm.Elbow.kD * Constants.Arm.Elbow.CoefficientMultiplier,
                 e_targetPosition = Constants.Arm.Elbow.defaultSetpoint;
                //  e_kIz = Constants.Arm.Elbow.kIz * Constants.Arm.Elbow.CoefficientMultiplier, 
                //  e_kFF = Constants.Arm.Elbow.kFF * Constants.Arm.Elbow.CoefficientMultiplier, 
                //  e_kMaxOutput = Constants.Arm.Elbow.kMaxOutput * Constants.Arm.Elbow.CoefficientMultiplier, 
                //  e_kMinOutput = Constants.Arm.Elbow.kMinOutput * Constants.Arm.Elbow.CoefficientMultiplier, 
  private double s_kP = Constants.Arm.Shoulder.kP * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kI = Constants.Arm.Shoulder.kI * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 s_kD = Constants.Arm.Shoulder.kD * Constants.Arm.Shoulder.CoefficientMultiplier,
                 s_targetPosition = Constants.Arm.Shoulder.defaultSetpoint;
                //  s_kIz = Constants.Arm.Shoulder.kIz * Constants.Arm.Shoulder.CoefficientMultiplier, 
                //  s_kFF = Constants.Arm.Shoulder.kFF * Constants.Arm.Shoulder.CoefficientMultiplier, 
                //  s_kMaxOutput = Constants.Arm.Shoulder.kMaxOutput * Constants.Arm.Shoulder.CoefficientMultiplier, 
                //  s_kMinOutput = Constants.Arm.Shoulder.kMinOutput * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 


  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    elbowMotor = new SendableCANSparkMax(Constants.Arm.Elbow.motor, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowMotor.setInverted(false);
    elbowMotor.setIdleMode(IdleMode.kBrake);

    shoulderMotor = new SendableCANSparkMax(Constants.Arm.Shoulder.motor, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(true);
    shoulderMotor.setIdleMode(IdleMode.kBrake);

    // elbowEncoder = new DutyCycleEncoder(Constants.Arm.Elbow.channel);
    // shoulderEncoder = new DutyCycleEncoder(Constants.Arm.Shoulder.channel);
    // elbowEncoder.setDistancePerRotation(360.0);
    // shoulderEncoder.setDistancePerRotation(360.0);
    

    shoulderRelativeEncoder = new FloorRelativeEncoder(Constants.Arm.Shoulder.channel, Constants.Arm.Shoulder.angleAtFloor, true);
    elbowRelativeEncoder = new FloorRelativeEncoder(Constants.Arm.Elbow.channel, Constants.Arm.Elbow.angleAtFloor, false, shoulderRelativeEncoder);
    // elbowEncoder.setAverageBits(4); 
    // shoulderEncoder.setAverageBits(4);

    elbowPidController = new PIDController(e_kP, e_kI, e_kD);
    shoulderPidController = new PIDController(s_kP, s_kI, s_kD);

    // elbowPidController.setSetpoint(e_targetPosition);
    // shoulderPidController.setSetpoint(s_targetPosition);

    //README: routing all setpoint updates through modifier functions
    setElbowSetpoint(Constants.Arm.Elbow.defaultSetpoint);
    setShoulderSetpoint(Constants.Arm.Shoulder.defaultSetpoint);

    // FIXME: need to characterize the elbow to find these values
    elbowFFController = new ArmFeedforward(Constants.Arm.Elbow.kS, Constants.Arm.Elbow.kG, Constants.Arm.Elbow.kV, Constants.Arm.Elbow.kA);
    shoulderFFController = new ArmFeedforward(Constants.Arm.Shoulder.kS, Constants.Arm.Shoulder.kG, Constants.Arm.Shoulder.kV, Constants.Arm.Shoulder.kA);
    // elbowPidController.setP(e_kP);
    // elbowPidController.setI(e_kI);
    // elbowPidController.setD(e_kD);
    // elbowPidController.setIZone(e_kIz);
    // elbowPidController.setFF(e_kFF);
    // elbowPidController.setOutputRange(e_kMinOutput, e_kMaxOutput);
    
    // shoulderPidController.setP(s_kP);
    // shoulderPidController.setI(s_kI);
    // shoulderPidController.setD(s_kD);
    // shoulderPidController.setIZone(s_kIz);
    // shoulderPidController.setFF(s_kFF);
    // shoulderPidController.setOutputRange(s_kMinOutput, s_kMaxOutput);
    
    //addChild("elbowMotor", elbowMotor);
    //addChild("shoulderMotor", shoulderMotor);

    shoulderRotationEntry = tab.add("Shoulder relative rotation", shoulderRelativeEncoder.getDegrees()).getEntry();
    elbowRotationEntry = tab.add("Elbow relative rotation", elbowRelativeEncoder.getDegrees()).getEntry();
    tab.add("Shoulder PID", shoulderPidController);
    tab.add("Elbow PID", elbowPidController);
    // tab.add("Shoulder Feedforward", shoulderFFController);
    // tab.add("Elbow Feedforward", elbowFFController);
    tab.add("Shoulder encoder", shoulderRelativeEncoder);
    tab.add("Elbow encoder",elbowRelativeEncoder);

    

    // shoulderPidController.setReference(shoulderEncoder.getAbsolutePosition() * 360, CANSparkMax.ControlType.kPosition);
    // elbowPidController.setReference(elbowEncoder.getAbsolutePosition() * 360, CANSparkMax.ControlType.kPosition);  


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
    return elbowRelativeEncoder.getDegrees() >= Constants.Arm.Elbow.upperlimit;
  }
  public boolean isElbowAtLowerLimit() {
    return elbowRelativeEncoder.getDegrees() <= Constants.Arm.Elbow.lowerlimit;
  }

  public boolean isShoulderAtUpperLimit() {
    return shoulderRelativeEncoder.getDegrees() >= Constants.Arm.Shoulder.upperlimit;
  }
  public boolean isShoulderAtLowerLimit() {
    return shoulderRelativeEncoder.getDegrees() <= Constants.Arm.Shoulder.lowerlimit;
  }

  // public void setElbowMotor(double speed) {
  //   elbowTargetSpeed = speed;
  //   //elbowMotor.set(speed);
  // }  
  // public void setShoulderMotor(double speed) {
  //   shoulderTargetSpeed = speed;
  //   //shoulderMotor.set(speed);
  // }

  // public void setElbowTarget(double angle) {
  //   angle = Math.min(angle, Constants.Arm.Elbow.upperlimit);
  //   angle = Math.max(angle, Constants.Arm.Elbow.lowerlimit);
  //   e_TargetRotation = angle;
  //   elbowPidController.setSetpoint(e_TargetRotation);
  //   System.out.println("e_angle: " + angle + "  e_TargetRotation: " + e_TargetRotation);
  // }

  // public void setShoulderTarget(double angle) {
  //   angle = Math.min(angle, Constants.Arm.Shoulder.upperlimit);
  //   angle = Math.max(angle, Constants.Arm.Shoulder.lowerlimit);
  //   s_TargetRotation = angle;
  //   shoulderPidController.setSetpoint(s_TargetRotation);
  //   System.out.println("s_angle: " + angle + "  s_TargetRotation: " + s_TargetRotation);
  // }

  // public void changeElbowTarget(double delta) {
  //   double angle = e_TargetRotation + delta;
  //   setElbowTarget(angle);
  // }

  // public void changeShoulderTarget(double delta) {
  //   double angle = s_TargetRotation + delta;
  //   setShoulderTarget(angle);
  // }

  // public double getRotationInDegree(DutyCycleEncoder encoder) {
  //   return encoder.getAbsolutePosition() * 360;
  // }

  // public double getElbowRotation() {
  //   return getRotationInDegree(elbowEncoder);
  // }

  // public double getShoulderRotation() {
  //   return getRotationInDegree(shoulderEncoder);
  // }

  
  public void setElbowSetpoint(double value) {
    //TODO: Review method definition
    e_targetPosition = MathUtil.clamp(value, Constants.Arm.Elbow.lowerlimit, Constants.Arm.Elbow.upperlimit);
    elbowPidController.setSetpoint(e_targetPosition);
  }

  public void changeElbowSetpoint(double delta) {
    setElbowSetpoint(e_targetPosition + delta);
  }

  public void setShoulderSetpoint(double value) {
    //TODO: Review method definition
    s_targetPosition = MathUtil.clamp(value, Constants.Arm.Shoulder.lowerlimit, Constants.Arm.Shoulder.upperlimit);
    shoulderPidController.setSetpoint(s_targetPosition);
  }

  public void changeShoulderSetpoint(double delta) {
    setShoulderSetpoint(s_targetPosition + delta);
  }

  /**
   * Get elbow's position relative to the floor.
   * @return Angle in degrees
   */
  public double getElbowPosition() {
    return elbowRelativeEncoder.getDegrees();
  }

  /**
   * Get shoulder's position relative to the floor.
   * @return Angle in degrees
   */
  public double getShoulderPosition() {
    return shoulderRelativeEncoder.getDegrees();
  }
  
  @Deprecated
  public void changeRelativeSetpoint(PIDController controller, double delta) {
    if (controller == elbowPidController) {
      e_targetPosition += delta;
      controller.setSetpoint(e_targetPosition + delta);
    }
    else {
      s_targetPosition += delta;
      controller.setSetpoint(s_targetPosition + delta);
    }
  }

  @Deprecated
  public void changeRelativeSetPoint(String name, double delta) {
    if (name.toLowerCase().equals("elbow")) {
      changeRelativeSetpoint(elbowPidController, delta);
    }
    else if (name.toLowerCase().equals("shoulder")) {
      changeRelativeSetpoint(shoulderPidController, delta);
    }
  }

  // public double getRelativeSetPoint(PIDController controller) {
  //   return controller.getSetpoint();
  // }

  // public double getRelativeSetPoint(String name) {
  //   if (name.toLowerCase().equals("elbow")) {
  //     getRelativeSetPoint(elbowPidController);
  //   }
  //   else if (name.toLowerCase().equals("shoulder")) {
  //     getRelativeSetPoint(shoulderPidController);
  //   }
  //   // In case of error
  //   return 0;
  // }

  // public double getShoulderSetPoint() {
  //   return shoulderPidController.getSetpoint();
  // }

  // public double getElbowSetPoint() {
  //   return elbowPidController.getSetpoint();
  // }


  public double calculateFeedForward(ArmFeedforward controller) {
    if (controller == elbowFFController) 
      return controller.calculate(e_targetPosition, Constants.Arm.Elbow.targetSpeed); 
    else 
      return controller.calculate(s_targetPosition, Constants.Arm.Shoulder.targetSpeed);
  }

 
  public double calculatePID(PIDController controller) {
    if (controller == elbowPidController) 
      return controller.calculate(elbowRelativeEncoder.getDegrees(), e_targetPosition);
    else 
      return controller.calculate(shoulderRelativeEncoder.getDegrees(), s_targetPosition);
  }

  // public void setShoulderTargetSpeed(double speed) {
  //   s_TargetVelocity = speed;
  // }

  // public void setElbowTargetSpeed(double speed) {
  //   e_TargetVelocity = speed;
  // }

  @Override
  public void periodic() {
    double elbowVoltage = 0;
    double shoulderVoltage = 0;

    if (Robot.isTestMode()){
      SmartDashboard.putBoolean("Elbow Lower Limit", isElbowAtLowerLimit());
      SmartDashboard.putBoolean("Elbow Upper Limit", isElbowAtUpperLimit());
      SmartDashboard.putBoolean("Shoulder Lower Limit", isShoulderAtLowerLimit());
      SmartDashboard.putBoolean("Shoulder Upper Limit", isShoulderAtUpperLimit());
      SmartDashboard.putNumber("Elbow Setpoint", e_targetPosition);
      SmartDashboard.putNumber("shoulder Setpoint", s_targetPosition);
      double e_p = elbowPidController.getP();
      double e_i = elbowPidController.getI();
      double e_d = elbowPidController.getD();
      //double e_Iz = elbowPidController.getIZone();
      //double e_FF = elbowPidController.getFF();
      //double e_max = elbowPidController.getOutputMax();
      //double e_min = elbowPidController.getOutputMin();
      //double e_CurrentRotation = elbowEncoder.getAbsolutePosition();

      double s_p = shoulderPidController.getP();
      double s_i = shoulderPidController.getI();
      double s_d = shoulderPidController.getD();
      //double s_Iz = shoulderPidController.getIZone();
      //double s_FF = shoulderPidController.getFF();
      //double s_max = shoulderPidController.getOutputMax();
      //double s_min = shoulderPidController.getOutputMin();
      //double s_CurrentRotation = shoulderEncoder.getAbsolutePosition();

      if (e_p != e_kP) {elbowPidController.setP(e_p); e_kP = e_p;}
      if (e_i != e_kI) {elbowPidController.setI(e_i); e_kI = e_i;}
      if (e_d != e_kD) {elbowPidController.setD(e_d); e_kD = e_d;}
      //if (e_Iz != e_kIz) {elbowPidController.setP(e_Iz); e_kIz = e_Iz;}
      //if (e_FF != e_kFF) {elbowPidController.setP(e_FF); e_kFF = e_FF;}
      //if (e_max != e_kMaxOutput || e_min != e_kMinOutput) {
        //elbowPidController.setOutputRange(e_min, e_max); 
        //e_kMaxOutput = e_max;
        //e_kMinOutput = e_min;
      //}
      
      if (s_p != s_kP) {shoulderPidController.setP(s_p); s_kP = s_p;}
      if (s_i != s_kI) {shoulderPidController.setI(s_i); s_kI = s_i;}
      if (s_d != s_kD) {shoulderPidController.setD(s_d); s_kD = s_d;}
      //if (s_Iz != s_kIz) {elbowPidController.setP(s_Iz); s_kIz = s_Iz;}
      //if (s_FF != s_kFF) {elbowPidController.setP(s_FF); s_kFF = s_FF;}
      //if (s_max != s_kMaxOutput || s_min != s_kMinOutput) {
      //  elbowPidController.setOutputRange(s_min, s_max); 
      //  s_kMaxOutput = s_max;
      //  s_kMinOutput = s_min;
      //}
    }

    else {

      elbowVoltage = calculatePID(elbowPidController) + calculateFeedForward(elbowFFController);
      shoulderVoltage = calculatePID(shoulderPidController) + calculateFeedForward(shoulderFFController);
      elbowVoltage = MathUtil.clamp(elbowVoltage, Constants.Arm.Elbow.minVoltage, Constants.Arm.Elbow.maxVoltage);
      shoulderVoltage = MathUtil.clamp(shoulderVoltage, Constants.Arm.Shoulder.minVoltage, Constants.Arm.Shoulder.maxVoltage);

      if(isElbowAtUpperLimit() && elbowVoltage > 0) {
        elbowVoltage = 0;
      }
      if(isShoulderAtUpperLimit() && shoulderVoltage > 0) {
        shoulderVoltage = 0;
      }
      
      if(isElbowAtLowerLimit() && elbowVoltage < 0) {
        elbowVoltage = 0;
      }
      if(isShoulderAtLowerLimit() && shoulderVoltage < 0) {
        shoulderVoltage = 0;
      }
    }

    elbowMotor.setVoltage(elbowVoltage);
    shoulderMotor.setVoltage(shoulderVoltage);

    elbowRotationEntry.setDouble(elbowRelativeEncoder.getDegrees());
    shoulderRotationEntry.setDouble(shoulderRelativeEncoder.getDegrees());

  //     if(isShoulderAtUpperLimit()){
  //       if (shoulderMotorSpeed > 0){
  //           shoulderMotor.stopMotor();
  //           shoulderMotorSpeed = 0;
  //       } 
  //   } 
  //   if (isshoulderAtLowerLimit()) {
  //       if (shoulderMotorSpeed < 0) {
  //         shoulderMotor.stopMotor(); 
  //         shoulderMotorSpeed = 0;
  //     }
  // }

  //   shoulderMotor.set(shoulderMotorSpeed);
  //   double elbowAdustSpeed = shoulderMotorSpeed * 0.595;
    

  //     if(isElbowAtUpperLimit()){
  //       if(elbowMotorSpeed > 0) {
  //         elbowMotor.stopMotor();    
  //         elbowMotorSpeed = 0;   
  //         if(shoulderMotorSpeed < 0)
  //           elbowAdustSpeed = 0;   
          
  //       }
  //   } 

  //   if (isElbowAtLowerLimit()) {
  //       if(elbowMotorSpeed < 0) {
  //         elbowMotor.stopMotor(); 
  //         elbowMotorSpeed = 0; 
  //         if(shoulderMotorSpeed > 0)
  //           elbowAdustSpeed = 0;
          
  //       }
  //   } 

      // This method will be called once per scheduler run

  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}

