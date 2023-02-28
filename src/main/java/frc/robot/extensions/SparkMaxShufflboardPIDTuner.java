package frc.robot.extensions;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Tuning.ApplySparkMaxShuffleboardTunerValuesCommand;

public class SparkMaxShufflboardPIDTuner {
    private String widgetName;
    private ShuffleboardTab tab;
    private SparkMaxPIDController controller;
    private PIDController tuner;

    public SparkMaxShufflboardPIDTuner(String tabName, String widgetName, int column, SparkMaxPIDController controller) {
        this.controller = controller;
        this.tuner = new PIDController(this.controller.getP(), this.controller.getI(), this.controller.getD());
        this.tab = Shuffleboard.getTab(tabName);
        this.tab.add(widgetName, tuner).withPosition(column, 0).withSize(2, 2);
        this.tab.add("Apply " + widgetName + " values", new ApplySparkMaxShuffleboardTunerValuesCommand(this)).withPosition(column, 2).withSize(2, 1);
    }

    public void applyTunerValues() {
        controller.setP(tuner.getP());
        controller.setI(tuner.getI());
        controller.setD(tuner.getD());
    }

    public void resetTunerValues() {
        tuner.setP(controller.getP());
        tuner.setI(controller.getI());
        tuner.setD(controller.getD());
    }



}
