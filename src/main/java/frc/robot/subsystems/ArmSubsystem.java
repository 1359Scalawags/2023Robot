// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.extensions.FloorRelativeEncoder;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.extensions.SparkMaxTuner;
import frc.robot.extensions.Utilities;

public class ArmSubsystem extends SubsystemBase {
  
  private SendableCANSparkMax elbowMotor;
  private SendableCANSparkMax shoulderMotor;

  private SparkMaxAbsoluteEncoder elbowSparkMaxEncoder;
  private SparkMaxAbsoluteEncoder shoulderSparkMaxEncoder;

  private SparkMaxTuner elbowTuner;
  private SparkMaxTuner shoulderTuner;

  private SparkMaxPIDController elbowSparkMaxPIDController;
  private SparkMaxPIDController shoulderSparkMaxPIDController;
  
  private GenericEntry shoulderRotationEntry;
  private GenericEntry elbowRotationEntry;

  private GenericEntry shoulderAbsoluteTargetEntry;
  private GenericEntry elbowAbsoluteTargetEntry;

  private SlewRateLimiter shouldRateLimiter;
  private SlewRateLimiter elbowRateLimiter;

  ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  Timer displayTimer = new Timer();

  private boolean isMovable = false;
  double lastDisplayTime = 0;

  private double e_kP = Constants.Arm.Elbow.kP,
                 e_kI = Constants.Arm.Elbow.kI, 
                 e_kD = Constants.Arm.Elbow.kD,
                 e_targetPosition = MathUtil.clamp(Constants.Arm.Elbow.defaultSetpoint, Constants.Arm.Elbow.lowerLimitWhenSafePos, Constants.Arm.Elbow.upperlimit),
                 e_lowerLimit = Constants.Arm.Elbow.lowerLimitWhenSafePos;
  private double s_kP = Constants.Arm.Shoulder.kP, 
                 s_kI = Constants.Arm.Shoulder.kI, 
                 s_kD = Constants.Arm.Shoulder.kD,
                 s_targetPosition = MathUtil.clamp(Constants.Arm.Shoulder.defaultSetpoint, Constants.Arm.Shoulder.lowerlimit, Constants.Arm.Shoulder.upperlimit);
                 
  private boolean isInitialized = false;

  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    displayTimer.start();
    lastDisplayTime = displayTimer.get();

    elbowMotor = new SendableCANSparkMax(Constants.Arm.Elbow.motor, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowMotor.setInverted(false);
    elbowMotor.setIdleMode(IdleMode.kBrake);
    
    // TODO: can/should we use setSoftLimit() to limit arm movement?
    elbowMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Arm.Elbow.upperlimit);
    elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Arm.Elbow.lowerLimitUnsafePosMin);
    
    shoulderMotor = new SendableCANSparkMax(Constants.Arm.Shoulder.motor, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(true);
    shoulderMotor.setIdleMode(IdleMode.kBrake);

    shoulderMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Arm.Shoulder.upperlimit);
    shoulderMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Arm.Shoulder.lowerlimit);

    elbowSparkMaxEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderSparkMaxEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    elbowSparkMaxEncoder.setInverted(false);
    shoulderSparkMaxEncoder.setInverted(true);
    elbowSparkMaxEncoder.setPositionConversionFactor(360.0);
    shoulderSparkMaxEncoder.setPositionConversionFactor(360.0);
    elbowSparkMaxEncoder.setZeroOffset(Constants.Arm.Elbow.angleAtFloor);
    shoulderSparkMaxEncoder.setZeroOffset(Constants.Arm.Shoulder.angleAtFloor);


    elbowSparkMaxPIDController = elbowMotor.getPIDController();
    shoulderSparkMaxPIDController = shoulderMotor.getPIDController();

    elbowSparkMaxPIDController.setP(Constants.Arm.Elbow.kP);
    elbowSparkMaxPIDController.setI(Constants.Arm.Elbow.kI);
    elbowSparkMaxPIDController.setD(Constants.Arm.Elbow.kD);
    elbowSparkMaxPIDController.setFF(Constants.Arm.Elbow.kFF);
    elbowSparkMaxPIDController.setIZone(Constants.Arm.Elbow.kIz);
    elbowSparkMaxPIDController.setOutputRange(Constants.Arm.Elbow.kMinOutput, Constants.Arm.Elbow.kMaxOutput);

    shoulderSparkMaxPIDController.setP(Constants.Arm.Shoulder.kP);
    shoulderSparkMaxPIDController.setI(Constants.Arm.Shoulder.kI);
    shoulderSparkMaxPIDController.setD(Constants.Arm.Shoulder.kD);
    shoulderSparkMaxPIDController.setFF(Constants.Arm.Shoulder.kFF);
    shoulderSparkMaxPIDController.setIZone(Constants.Arm.Shoulder.kIz);
    shoulderSparkMaxPIDController.setOutputRange(Constants.Arm.Shoulder.kMinOutput, Constants.Arm.Shoulder.kMaxOutput);

    shoulderRotationEntry = tab.add("Shoulder relative rotation", getShoulderDegree())
                            .withSize(2, 1).withPosition(2, 0).getEntry();
    elbowRotationEntry = tab.add("Elbow relative rotation", getElbowDegree())
                         .withSize(2, 1).withPosition(2, 1).getEntry();

    shoulderAbsoluteTargetEntry = tab.add("Shoulder absolute target", s_targetPosition)
                                  .withSize(2, 1).withPosition(0, 0).getEntry();
    elbowAbsoluteTargetEntry = tab.add("Elbow absolute target", e_targetPosition)
                               .withSize(2, 1).withPosition(0, 1).getEntry();

    elbowTuner = new SparkMaxTuner("SparkMax Tuner", "Elbow Tuner", 0, elbowSparkMaxPIDController);
    shoulderTuner = new SparkMaxTuner("SparkMax Tuner", "Shoulder Tuner", 2, shoulderSparkMaxPIDController);
            
    //elbowRateLimiter = new SlewRateLimiter(Constants.Arm.Elbow.slewRateLimiter * Constants.Arm.rateLimiterMultiplier);
    //shouldRateLimiter = new SlewRateLimiter(Constants.Arm.Shoulder.slewRateLimiter * Constants.Arm.rateLimiterMultiplier);
    initializeSetpoints();
  
  }


  public void initializeEncoders() {
    getShoulderDegree();
    getElbowDegree();
  }
  /**
   * Initialize setpoints at current arm joint positions.
   */
  public void initializeSetpoints() {
    elbowRateLimiter = new SlewRateLimiter(Constants.Arm.Elbow.slewRateLimiter * Constants.Arm.rateLimiterMultiplier, -Constants.Arm.Elbow.slewRateLimiter * Constants.Arm.rateLimiterMultiplier, getElbowDegree());
    shouldRateLimiter = new SlewRateLimiter(Constants.Arm.Shoulder.slewRateLimiter * Constants.Arm.rateLimiterMultiplier);
    
    //shouldRateLimiter.reset(getElbowDegree());  <<<<<-----This was a problem
    shouldRateLimiter.reset(getShoulderDegree());
    isMovable = true;    
    
    setElbowSetpoint(getElbowDegree());
    setShoulderSetpoint(getShoulderDegree());
    System.out.println("Initial Setpoints: E:" + e_targetPosition + " S:" + s_targetPosition);
    isInitialized = true;

  }

  // TODO: Figure out how to change encoders without breaking things?
  public boolean isElbowAtUpperLimit() {
    return getElbowDegree() >= Constants.Arm.Elbow.upperlimit;
  }
  public boolean isElbowAtLowerLimit() {
    return getElbowDegree() <= e_lowerLimit;
  }
  public boolean isElbowWithinLimits() {
    return !isElbowAtLowerLimit() && !isElbowAtUpperLimit();
  }
  public boolean isElbowAtUnsafe() {
    double elbowDegree = getElbowDegree();
    return elbowDegree <= Constants.Arm.Elbow.lowerLimitWhenSafePos;
  }

  public boolean isShoulderAtUpperLimit() {
    return getShoulderDegree() >= Constants.Arm.Shoulder.upperlimit;
  }
  public boolean isShoulderAtUpperLimit(double buffer) {
    return getShoulderDegree() >= Constants.Arm.Shoulder.upperlimit + buffer;
  }
  public boolean isShoulderAtLowerLimit() {
    return getShoulderDegree() <= Constants.Arm.Shoulder.lowerlimit;
  }
  public boolean isShoulderAtLowerLimit(double buffer) {
    return getShoulderDegree() <= Constants.Arm.Shoulder.lowerlimit - buffer;
  }
  public boolean isShoulderWithinLimits() {
    return !isShoulderAtLowerLimit() && !isShoulderAtUpperLimit();
  }

  // public void setPinAutonomous() {
  //   elbowSparkMaxPIDController.setP(Constants.Arm.Elbow.kPAuto);
  //   shoulderSparkMaxPIDController.setP(Constants.Arm.Shoulder.kPAuto);
  // }

  public void setPinTeleop() {
    elbowSparkMaxPIDController.setP(Constants.Arm.Elbow.kP);
    shoulderSparkMaxPIDController.setP(Constants.Arm.Shoulder.kP);
  }

  /**
   * Set target position for elbow and its controllers.
   * The value will be restricted within the elbow's physical limits.
   * @param value Target angle in degrees relative to the floor.
   */
  public void setElbowSetpoint(double value) {
    adjustElbowLimit();
    //TODO: Review method definition
    if (isMovable) {
      e_targetPosition = MathUtil.clamp(value, e_lowerLimit, Constants.Arm.Elbow.upperlimit);
    }
   
    //elbowSparkMaxPIDController.setReference(elbowRelativeEncoder.convertToAbsoluteDegrees(e_targetPosition), ControlType.kPosition);
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
    if (isMovable) {
      s_targetPosition = MathUtil.clamp(value, Constants.Arm.Shoulder.lowerlimit, Constants.Arm.Shoulder.upperlimit);
    }
    //shoulderSparkMaxPIDController.setReference(shoulderRelativeEncoder.convertToAbsoluteDegrees(s_targetPosition), ControlType.kPosition);
  }

    /**
   * Change the shoulder's target position.
   * The resulting target will be restricted within the elbow's physical limits.
   * @param delta Amount to change the position in degrees.
   */
  public void changeShoulderSetpoint(double delta) {
    setShoulderSetpoint(s_targetPosition + delta);
  }

  public void parkShoulder() {
    isMovable = false;
    s_targetPosition = Constants.Arm.Shoulder.parkingDegree;
  }

  public void parkElbow() {
    isMovable = false;
    e_targetPosition = Constants.Arm.Elbow.parkingDegree;
  }

  public void unParkShoulder() {
    isMovable = false;
    s_targetPosition = Constants.Arm.Shoulder.unParkingDegree;
  }

  public void unParkElbow() {
    isMovable = false;
    e_targetPosition = Constants.Arm.Elbow.unParkingDegree;
  }
  // /**
  //  * Get elbow's position relative to the floor.
  //  * @return Angle in degrees
  //  */
  // public double getElbowPosition() {
  //   return elbowRelativeEncoder.getDegrees();
  // }


  // public double getShoulderPosition() {
  //   return shoulderRelativeEncoder.getDegrees();
  // }

  /**
   * Get shoulder's position relative to the floor.
   * @return Angle in degrees
   */
  public double getShoulderDegree() {
    return shoulderSparkMaxEncoder.getPosition();
  }
  
  /**
   * The floor relative angle of the elbow.
   * @return Angle in degrees.
   */

  public double getElbowDegree() {
    double offset = Constants.Arm.Elbow.angleAtFloor - getShoulderDegree() + 360;
    double normalized = MathUtil.inputModulus(offset, 0, 360);
    //avoid spam messages
    REVLibError error = elbowSparkMaxEncoder.setZeroOffset(normalized);
    if(displayTimer.hasElapsed(0.5)) {
       displayTimer.reset();
    //   System.out.println("Offset: " + offset + "    Normalized: " + normalized);      
      
      if(error != REVLibError.kOk) {
          System.out.println("Unable to use " + normalized + " degrees as an offset.");   
      }      
    }
    return elbowSparkMaxEncoder.getPosition() ;
  }

  public boolean isElbowAtTarget(double target, double tolerance) {
    if(Math.abs(target - getElbowDegree()) < tolerance) {
        return true;
    }
    return false;
  }

  public boolean isShoulderAtTarget(double target, double tolerance) {
    if(Math.abs(target - getShoulderDegree()) < tolerance) {
        return true;
    }
    return false;
  }

  private double CalcElbowLower(double x){
    x+=Constants.Arm.triggerZone;//ElbowTriggerZone;

    double b = 0.0000033922;
    double c = -1.46987;
    double d = 427.188;
    return (b * Math.pow(x, 3) + c * x  + d);
  }

  public void adjustElbowLimit() {
    // double shoulder = getShoulderDegree();
    // if (Utilities.IsCloseTo(shoulder, Constants.Arm.Shoulder.upperlimit, 3.0)) {
    //   e_lowerLimit = Constants.Arm.Elbow.lowerLimitUnsafePosMin;
    //   if (isElbowAtUnsafe()) {
    //     s_targetPosition = Constants.Arm.Shoulder.upperlimit;
    //   }
    // }
    // else if (Utilities.isGreaterThan(shoulder, Constants.Arm.Shoulder.safelimit, 3.0)){
    //   e_lowerLimit = Constants.Arm.Elbow.lowerLimitWhenSafePos;
    // }
    // else {
    //   e_lowerLimit = Constants.Arm.Elbow.lowerLimitWhenShoulderSafe;
    // }

    double ShouldDegree = getShoulderDegree();
    if (ShouldDegree <= 246 && ShouldDegree >= 196 ){//&& getElbowDegree() > 45) {
      e_lowerLimit = CalcElbowLower(ShouldDegree);
    }
    else if (ShouldDegree > 246) {
      e_lowerLimit =  118;
    }
    else if (ShouldDegree < 196){
      e_lowerLimit = 164;}
    }

//   public void adjustElbowLimit() {
//     return Constants.Arm.Elbow.lowerLimitWhenSafePos;
//     double shoulder = getShoulderDegree();
//     if (Utilities.IsCloseTo(shoulder, Constants.Arm.Shoulder.upperlimit, 3.0)) {
//       e_lowerLimit = Constants.Arm.Elbow.lowerLimitUnsafePosMin;
//       if (isElbowAtUnsafe()) {
//         setShoulderSetpoint(Constants.Arm.Shoulder.upperlimit);;
//       }
//     }
//     else if (Utilities.isGreaterThan(shoulder, Constants.Arm.Shoulder.safelimit, 3.0)){
//       e_lowerLimit = Constants.Arm.Elbow.lowerLimitWhenSafePos;
//     }
//     else {
//       e_lowerLimit = Constants.Arm.Elbow.lowerLimitWhenShoulderSafe;
//     }
  
//   //   if(shoulder < Constants.Arm.Elbow.shoulderRestrictionPositionLower) {
//   //     e_lowerLimit = 90.0;
//   //   } else if(shoulder > Constants.Arm.Elbow.shoulderRestrictionPositionUpper) {
//   //     e_lowerLimit = 115;
//   //   } else {
//   //     e_lowerLimit = 0.000626961 * Math.pow(shoulder, 3) - 0.457477 * Math.pow(shoulder, 2) + 111.372 * shoulder - 8945.89;      
//   //   }
//   //   e_targetPosition = MathUtil.clamp(e_targetPosition, e_lowerLimit, Constants.Arm.Elbow.upperlimit);
//   //   elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)e_lowerLimit);
//   // }
//   }

  public void setFlagForMovingArm(boolean choice) {
    isMovable = choice;
  } 

  int delayCounter = 0;
  int counter = 0;
  @Override
  public void periodic() {
    if(delayCounter < 100) {
      delayCounter++;
      return;
    }
    //TODO: Check if the Smartdashboard addon is important
    if (Robot.isTestMode()){
      SmartDashboard.putBoolean("Elbow Lower Limit", isElbowAtLowerLimit());
      SmartDashboard.putBoolean("Elbow Upper Limit", isElbowAtUpperLimit());
      SmartDashboard.putBoolean("Shoulder Lower Limit", isShoulderAtLowerLimit());
      SmartDashboard.putBoolean("Shoulder Upper Limit", isShoulderAtUpperLimit());
      SmartDashboard.putNumber("Elbow Setpoint", e_targetPosition);
      SmartDashboard.putNumber("shoulder Setpoint", s_targetPosition);
    }

    else {
      if(isInitialized) {
          //adjustElbowLimit();
          elbowSparkMaxPIDController.setReference(elbowRateLimiter.calculate(e_targetPosition), ControlType.kPosition);
          shoulderSparkMaxPIDController.setReference(shouldRateLimiter.calculate(s_targetPosition), ControlType.kPosition);
      
        }
    }

    if (counter > Constants.UI.delayCounter) {
      shoulderRotationEntry.setDouble(getShoulderDegree());
      elbowRotationEntry.setDouble(getElbowDegree());

      shoulderAbsoluteTargetEntry.setDouble(s_targetPosition);
      elbowAbsoluteTargetEntry.setDouble(e_targetPosition);
      counter = 0;
    }
    counter++;

      // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}

