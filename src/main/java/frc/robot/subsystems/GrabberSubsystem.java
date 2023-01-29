// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.REVPHDataJNI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
 

  /** Creates a new ExampleSubsystem. */
  Solenoid Open;
  Solenoid Closed;
  boolean grabberOpen;
  Compressor phCompressor;
  public GrabberSubsystem() {
    Compressor phCompressor = new Compressor(Constants.SwerveDrive.Arm.grabber.compressorModule, PneumaticsModuleType.REVPH);
    phCompressor.enableDigital();
    Open = new Solenoid(Constants.SwerveDrive.Arm.grabber.openSolenoidModuleA, PneumaticsModuleType.REVPH, Constants.SwerveDrive.Arm.grabber.openSolenoidModuleB);
    Closed = new Solenoid(Constants.SwerveDrive.Arm.grabber.closedSolenoidModuleA, PneumaticsModuleType.REVPH, Constants.SwerveDrive.Arm.grabber.closedSolenoidModuleB);
}
  //TODO: what are we using and where is it going
  //Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
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
  public void TurnOff(){
    phCompressor.disable();
  }
  public void TurnOn(){
    phCompressor.enableDigital();
  }

  public boolean isOpen(){
    return grabberOpen;
  }
  public void close(){
    Open.set(false);
    Closed.set(true);
    grabberOpen = false;
  }
  public void open(){
    Open.set(true);
    Closed.set(false);
    grabberOpen = true;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
