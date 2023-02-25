package frc.robot.extensions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScaledFloorRelativeTuner {
    
    private PIDController controller;
    private FloorRelativeEncoder encoder;
    private ShuffleboardTab tunerTab;
    private GenericEntry pEntry;
    private GenericEntry iEntry;
    private GenericEntry dEntry;
    private GenericEntry actualEntry;
    private GenericEntry targetEntry;
    private GenericEntry errorEntry;
    private double kP, kI, kD, targetValue, actualValue, errorValue;
    
    public ScaledFloorRelativeTuner(PIDController controller, FloorRelativeEncoder encoder) {
        this.controller = controller;
        this.encoder = encoder;
    }

    private void syncEntries() {
        double p = pEntry.getDouble(0);
        double i = iEntry.getDouble(0);
        double d = dEntry.getDouble(0);
        double target = SmartDashboard.getNumber("Target", 0);
        if((p != kP)) { controller.setP(p); kP = p; }
        if((i != kI)) { controller.setI(i); kI = i; }
        if((d != kD)) { controller.setD(d); kD = d; }
        if(target != targetValue) {controller.setSetpoint(target); targetValue = target;}

        actualValue = encoder.getDegrees();
        errorValue = targetValue - actualValue;

        actualEntry.setDouble(actualValue);
        errorEntry.setDouble(errorValue);
        //System.out.println("Actual: " + actualValue + " Error: " + errorValue);
    }

    private void layoutTabs(ShuffleboardTab shuffleTab) {
        pEntry = shuffleTab
                    .add("P", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(1,1)
                    .withPosition(0, 0)
                    .getEntry();

        iEntry = shuffleTab
                    .add("I", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(1,1)
                    .withPosition(1, 0)
                    .getEntry();

        dEntry = shuffleTab
                    .add("D", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(1,1)
                    .withPosition(2, 0)
                    .getEntry();

        targetEntry = shuffleTab
                    .add("Target", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(1,1)
                    .withPosition(1, 1)
                    .getEntry();

        actualEntry = shuffleTab
                    .add("Actual", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(1,1)
                    .withPosition(2, 1)
                    .getEntry();

        errorEntry = shuffleTab
                    .add("Error", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(1,1)
                    .withPosition(3, 1)
                    .getEntry();                    
    }


    public void periodic() {
        syncEntries();
        double p = SmartDashboard.getNumber("P", 0);
        double i = SmartDashboard.getNumber("I", 0);
        double d = SmartDashboard.getNumber("D", 0);
        double target = SmartDashboard.getNumber("Target", 0);

        if((p != kP)) { controller.setP(p); kP = p; }
        if((i != kI)) { controller.setI(i); kI = i; }
        if((d != kD)) { controller.setD(d); kD = d; }
        if(target != targetValue) {controller.setSetpoint(target); targetValue = target;}

        actualValue = encoder.getDegrees();
        errorValue = targetValue - actualValue;
        SmartDashboard.putNumber("Actual", actualValue);
        SmartDashboard.putNumber("Error", errorValue);
        //System.out.println("Target: " + targetValue + "  Actual: " + actualValue + "  Error: " + errorValue);
    }



}
