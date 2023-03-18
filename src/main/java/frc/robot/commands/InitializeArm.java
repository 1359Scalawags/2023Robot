// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class InitializeArm extends CommandBase {

  private final ArmSubsystem subsystem;
  private boolean encodersReady;
  private boolean setpointsReady;
  private Timer waitTimer;

  private final static double WAIT_TIME = 0.25;

  public InitializeArm(ArmSubsystem subsystem) {
    this.subsystem = subsystem;
    this.encodersReady = false;
    this.setpointsReady = false;
    this.waitTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTimer.reset();
    waitTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //initialize encoders first after wait time
    if(!encodersReady) {
      if(waitTimer.get() > WAIT_TIME) {
        subsystem.initializeEncoders();
        waitTimer.restart();
        encodersReady = true;        
      }
    }
    // initialize setpoints after encoders are initialized
    else {
      if(waitTimer.get() > WAIT_TIME) {
        subsystem.initializeSetpoints();
        setpointsReady = true;        
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (encodersReady && setpointsReady);
  }
}
