// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisplaySubsystem extends SubsystemBase {
  private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private GenericEntry timeEntry;
  private int counter = 0;
  /** Creates a new ExampleSubsystem. */
  public DisplaySubsystem(VisionSystem vision) {
    Shuffleboard.selectTab("Main");
    timeEntry = mainTab.add("Match time", 0).getEntry();  
    mainTab.add("Camera", vision.getCamera1());
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (counter > 5){
      timeEntry.setDouble(DriverStation.getMatchTime());
    }
    counter++;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
