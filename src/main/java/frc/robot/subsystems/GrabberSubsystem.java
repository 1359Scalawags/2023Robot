// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
 

  /** Creates a new ExampleSubsystem. */
  boolean grabberOpen;
  Compressor phCompressor;
  DoubleSolenoid m_doubleSolenoid1;
  DoubleSolenoid m_doubleSolenoid2;
  PneumaticHub m_pH;
  // public GrabberSubsystem() {
  //   phCompressor = new Compressor(Constants.Grabber.compressorModule, PneumaticsModuleType.REVPH);
  //   phCompressor.enableDigital();
  //  DoubleSolenoid m_doublesolenoid;
  //  m_doublesolenoid = m_pH.makeDoubleSolenoid(1,0);

  public GrabberSubsystem() {
    phCompressor = new Compressor(Constants.Grabber.compressorModule, PneumaticsModuleType.REVPH);
    m_pH = new PneumaticHub(Constants.Grabber.Pneumatic.PneumaticHub);
    m_doubleSolenoid1 = m_pH.makeDoubleSolenoid(Constants.Grabber.closedSolenoidModuleA,Constants.Grabber.openSolenoidModuleA);
    m_doubleSolenoid2 = m_pH.makeDoubleSolenoid(Constants.Grabber.closedSolenoidModuleB,Constants.Grabber.openSolenoidModuleB);
    close();
  
  }

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
    m_doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
    m_doubleSolenoid2.set(DoubleSolenoid.Value.kForward);
    grabberOpen = false;
  }
  public void open(){
    m_doubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
    m_doubleSolenoid2.set(DoubleSolenoid.Value.kReverse);
    grabberOpen = true;
  }
  public boolean isCompressorOn(){
    return phCompressor.isEnabled();
  }
  public void reverseState(){
      if(this.isOpen()){
        this.close();
      }
      else{
        this.open();
      }
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
