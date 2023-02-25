// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Tuning.ApplyShoulderTuningCommand;
import frc.robot.commands.Tuning.ApplyElbowSGVTuningCommand;
import frc.robot.commands.Tuning.ApplyShoulderSGVTuningCommand;
import frc.robot.commands.Tuning.ApplyElbowPIDTuningCommand;
import frc.robot.extensions.FloorRelativeEncoder;
import frc.robot.extensions.SendableCANSparkMax;

public class ArmSubsystem extends SubsystemBase {
  
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;
  //private DutyCycleEncoder elbowEncoder;
  //private DutyCycleEncoder shoulderEncoder;
  private FloorRelativeEncoder elbowRelativeEncoder;
  private FloorRelativeEncoder shoulderRelativeEncoder;

  private ProfiledPIDController elbowPidController;
  private ProfiledPIDController shoulderPidController;

  private PIDController elbowTuner;
  private PIDController shoulderTuner;

  private ArmFeedforward elbowFFController;
  private ArmFeedforward shoulderFFController;

  private PIDController elbowSGVTuner;
  private PIDController shoulderSGVTuner;
  
  private GenericEntry shoulderRotationEntry;
  private GenericEntry elbowRotationEntry;

  ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  // private double elbowTargetSpeed = 0;
  // private double shoulderTargetSpeed = 0;

  private double e_kP = Constants.Arm.Elbow.kP,
                 e_kI = Constants.Arm.Elbow.kI, 
                 e_kD = Constants.Arm.Elbow.kD,
                 e_targetPosition = Constants.Arm.Elbow.defaultSetpoint;
                //  e_kIz = Constants.Arm.Elbow.kIz * Constants.Arm.Elbow.CoefficientMultiplier, 
                //  e_kFF = Constants.Arm.Elbow.kFF * Constants.Arm.Elbow.CoefficientMultiplier, 
                //  e_kMaxOutput = Constants.Arm.Elbow.kMaxOutput * Constants.Arm.Elbow.CoefficientMultiplier, 
                //  e_kMinOutput = Constants.Arm.Elbow.kMinOutput * Constants.Arm.Elbow.CoefficientMultiplier, 
  private double s_kP = Constants.Arm.Shoulder.kP, 
                 s_kI = Constants.Arm.Shoulder.kI, 
                 s_kD = Constants.Arm.Shoulder.kD,
                 s_targetPosition = Constants.Arm.Shoulder.defaultSetpoint;
                //  s_kIz = Constants.Arm.Shoulder.kIz * Constants.Arm.Shoulder.CoefficientMultiplier, 
                //  s_kFF = Constants.Arm.Shoulder.kFF * Constants.Arm.Shoulder.CoefficientMultiplier, 
                //  s_kMaxOutput = Constants.Arm.Shoulder.kMaxOutput * Constants.Arm.Shoulder.CoefficientMultiplier, 
                //  s_kMinOutput = Constants.Arm.Shoulder.kMinOutput * Constants.Arm.Shoulder.CoefficientMultiplier, 
                 
  private boolean initialized = false;

  
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

    elbowPidController = new ProfiledPIDController(e_kP, e_kI, e_kD, new TrapezoidProfile.Constraints(Constants.Arm.Elbow.MaxVelocity, Constants.Arm.Elbow.MaxAcceleration));
    shoulderPidController = new ProfiledPIDController(s_kP, s_kI, s_kD, new TrapezoidProfile.Constraints(Constants.Arm.Shoulder.MaxVelocity, Constants.Arm.Shoulder.MaxAcceleration));

    elbowTuner = new PIDController(e_kP, e_kI, e_kD);
    shoulderTuner = new PIDController(s_kP, s_kI, s_kD);

    // elbowPidController.setSetpoint(e_targetPosition);
    // shoulderPidController.setSetpoint(s_targetPosition);

    //README: routing all setpoint updates through modifier functions
    setElbowSetpoint(Constants.Arm.Elbow.defaultSetpoint);
    setShoulderSetpoint(Constants.Arm.Shoulder.defaultSetpoint);

    // FIXME: need to characterize the elbow to find these values
    elbowFFController = new ArmFeedforward(Constants.Arm.Elbow.kS, Constants.Arm.Elbow.kG, Constants.Arm.Elbow.kV, Constants.Arm.Elbow.kA);
    shoulderFFController = new ArmFeedforward(Constants.Arm.Shoulder.kS, Constants.Arm.Shoulder.kG, Constants.Arm.Shoulder.kV, Constants.Arm.Shoulder.kA);

    elbowSGVTuner = new PIDController(Constants.Arm.Elbow.kS, Constants.Arm.Elbow.kG, Constants.Arm.Elbow.kV);
    shoulderSGVTuner = new PIDController(Constants.Arm.Shoulder.kS, Constants.Arm.Shoulder.kG, Constants.Arm.Shoulder.kV);
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
    tab.add("Shoulder PID", shoulderPidController).withPosition(0, 0).withSize(2, 2);
    tab.add("Elbow PID", elbowPidController).withWidget(BuiltInWidgets.kPIDController).withPosition(2, 0).withSize(2, 2);
    
    tab.add("Shoulder Tuner", shoulderTuner).withPosition(0, 2).withSize(2, 2);
    tab.add("Apply Shoulder Tuning Values", new ApplyShoulderTuningCommand(this)).withPosition(10, 0).withSize(2, 1);
    
    tab.add("Shoulder SGV Tuner", shoulderSGVTuner).withPosition(0, 4).withSize(2, 2);
    tab.add("Apply Shoulder SGV Values", new ApplyShoulderSGVTuningCommand(this)).withPosition(10, 2).withSize(2, 1);

    tab.add("Elbow PID Tuner", elbowTuner).withPosition(2, 2).withSize(2, 2);
    tab.add("Apply Elbow PID Tuning Values", new ApplyElbowPIDTuningCommand(this)).withPosition(10, 0).withSize(2, 1);

    tab.add("Elbow SGV Tuner", elbowSGVTuner).withPosition(4, 0).withSize(2,2);
    tab.add("Apply Elbow SGV Values", new ApplyElbowSGVTuningCommand(this)).withPosition(10, 4).withSize(2, 1);
    // tab.add("Shoulder Feedforward", shoulderFFController);
    // tab.add("Elbow Feedforward", elbowFFController);
    //tab.add("Shoulder encoder", shoulderRelativeEncoder);
    //tab.add("Elbow encoder",elbowRelativeEncoder);

            
  }

  /**
   * Initialize setpoints at current arm joint positions.
   * @param elbow Set elbow setpoint.
   * @param shoulder Set shoulder setpoint.
   */
  public void initializeSetpoints(boolean elbow, boolean shoulder) {
    initialized = false;
    if(elbow) {
      setElbowSetpoint(elbowRelativeEncoder.getDegrees());
    }
    if(shoulder) {
      setShoulderSetpoint(shoulderRelativeEncoder.getDegrees());
    }
    System.out.println("Initial Setpoints: E:" + e_targetPosition + " S:" + s_targetPosition);
  }

  /**
   * Initialize both elbow and shoulder setpoints at current joint positions.
   */
  public void initializeSetpoints() {
    initializeSetpoints(true, true);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /**
   * Update PID values from the Tuner.
   */

  public void applyElbowTuningValues() {
    elbowPidController.setP(elbowTuner.getP());
    elbowPidController.setI(elbowTuner.getI());
    elbowPidController.setD(elbowTuner.getD());
    System.out.println("==========================");
    System.out.println("Elbow PID Controller Updated");
    System.out.println("==========================");
  }

  public void applyElbowSGVTuningValues() {
    elbowFFController = new ArmFeedforward(elbowSGVTuner.getP(), elbowSGVTuner.getI(), elbowSGVTuner.getD(), Constants.Arm.Elbow.kA);
    System.out.println("==========================");
    System.out.println("Elbow FF Controller Updated");
    System.out.println("==========================");
  }

  public void applyShoulderTuningValues() {
    shoulderPidController.setP(shoulderTuner.getP());
    shoulderPidController.setI(shoulderTuner.getI());
    shoulderPidController.setD(shoulderTuner.getD());
    System.out.println("==========================");
    System.out.println("Shoulder PID Controller Updated");
    System.out.println("==========================");
  }

  public void applyShoulderSGVTuningValues() {
    elbowFFController = new ArmFeedforward(shoulderSGVTuner.getP(), shoulderSGVTuner.getI(), shoulderSGVTuner.getD(), Constants.Arm.Shoulder.kA);
    System.out.println("==========================");
    System.out.println("Shoulder FF Controller Updated");
    System.out.println("==========================");
  }


  public boolean isElbowAtUpperLimit() {
    return elbowRelativeEncoder.getDegrees() >= Constants.Arm.Elbow.upperlimit;
  }
  public boolean isElbowAtLowerLimit() {
    return elbowRelativeEncoder.getDegrees() <= Constants.Arm.Elbow.lowerlimit;
  }
  public boolean isElbowWithinLimits() {
    return !isElbowAtLowerLimit() && !isElbowAtUpperLimit();
  }

  public boolean isShoulderAtUpperLimit() {
    return shoulderRelativeEncoder.getDegrees() >= Constants.Arm.Shoulder.upperlimit;
  }
  public boolean isShoulderAtUpperLimit(double buffer) {
    return shoulderRelativeEncoder.getDegrees() >= Constants.Arm.Shoulder.upperlimit + buffer;
  }
  public boolean isShoulderAtLowerLimit() {
    return shoulderRelativeEncoder.getDegrees() <= Constants.Arm.Shoulder.lowerlimit;
  }
  public boolean isShoulderAtLowerLimit(double buffer) {
    return shoulderRelativeEncoder.getDegrees() <= Constants.Arm.Shoulder.lowerlimit - buffer;
  }
  public boolean isShoulderWithinLimits() {
    return !isShoulderAtLowerLimit() && !isShoulderAtUpperLimit();
  }

  /**
   * Set target position for elbow and its controllers.
   * The value will be restricted within the elbow's physical limits.
   * @param value Target angle in degrees relative to the floor.
   */
  public void setElbowSetpoint(double value) {
    //TODO: Review method definition
    e_targetPosition = MathUtil.clamp(value, Constants.Arm.Elbow.lowerlimit, Constants.Arm.Elbow.upperlimit);
    elbowPidController.setGoal(s_targetPosition);
  }

  /**
   * Change the elbow's target position.
   * The resulting target will be restricted within the elbow's physical limits.
   * @param delta Amount to change the position in degrees.
   */
  public void changeElbowSetpoint(double delta) {
    setElbowSetpoint(e_targetPosition + delta);
  }

    /**
   * Set target position for shoulder and its controllers.
   * The value will be restricted within the shoulder's physical limits.
   * @param value Target angle in degrees relative to the floor.
   */
  public void setShoulderSetpoint(double value) {
    //TODO: Review method definition
    s_targetPosition = MathUtil.clamp(value, Constants.Arm.Shoulder.lowerlimit, Constants.Arm.Shoulder.upperlimit);
    shoulderPidController.setGoal(s_targetPosition);
  }

    /**
   * Change the shoulder's target position.
   * The resulting target will be restricted within the elbow's physical limits.
   * @param delta Amount to change the position in degrees.
   */
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
  

  public double getElbowFF() {
      return elbowFFController.calculate(e_targetPosition * Math.PI / 180.0, Constants.Arm.Elbow.targetSpeed); 
  }

  public double getShoulderFF() {
      return shoulderFFController.calculate(s_targetPosition * Math.PI / 180.0, Constants.Arm.Shoulder.targetSpeed);
  }

  public double getElbowPID() {
    return elbowPidController.calculate(elbowRelativeEncoder.getDegrees(), e_targetPosition);
  }

  public double getShoulderPID() {
    return shoulderPidController.calculate(shoulderRelativeEncoder.getDegrees(), s_targetPosition);    
  }


  int delayCounter = 0;
  @Override
  public void periodic() {
    if(delayCounter < 100) {
      delayCounter++;
      return;
    }
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

      elbowVoltage = getElbowPID() + getElbowFF();
      shoulderVoltage = getShoulderPID() + getShoulderFF();
      elbowVoltage = MathUtil.clamp(elbowVoltage, Constants.Arm.Elbow.minVoltage, Constants.Arm.Elbow.maxVoltage);
      shoulderVoltage = MathUtil.clamp(shoulderVoltage, Constants.Arm.Shoulder.minVoltage, Constants.Arm.Shoulder.maxVoltage);

      //slowdown assembly during startup if not within limits
      if(initialized == false) {
        if(isElbowWithinLimits() && isShoulderWithinLimits()) {
          initialized = true;
        } else {
          elbowVoltage *= Constants.Arm.softStartRatio;
          shoulderVoltage *= Constants.Arm.softStartRatio;          
        }
      }

      if(isElbowAtUpperLimit() && elbowVoltage > 0) {
        elbowVoltage = 0;
      }
      if(isShoulderAtUpperLimit(Constants.Arm.boundaryExtension) && shoulderVoltage > 0) {
        shoulderVoltage = 0;
      }
      
      if(isElbowAtLowerLimit() && elbowVoltage < 0) {
        elbowVoltage = 0;
      }
      if(isShoulderAtLowerLimit(Constants.Arm.boundaryExtension) && shoulderVoltage < 0) {
        shoulderVoltage = 0;
      }
    }

    elbowMotor.setVoltage(elbowVoltage);
    shoulderMotor.setVoltage(shoulderVoltage);

    if(elbowRotationEntry.getDouble(0) != elbowRelativeEncoder.getDegrees()) {
      elbowRotationEntry.setDouble(elbowRelativeEncoder.getDegrees());      
    }
    if(shoulderRotationEntry.getDouble(0) != shoulderRelativeEncoder.getDegrees()) {
      shoulderRotationEntry.setDouble(shoulderRelativeEncoder.getDegrees());      
    }


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

