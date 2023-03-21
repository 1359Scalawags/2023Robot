package frc.robot.extensions;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SparkMaxTuner {
    private ShuffleboardTab tab;
    private SparkMaxPIDController controller;
    private PIDController tuner;

    public SparkMaxTuner(String tabName, String widgetName, int column, SparkMaxPIDController controller) {
        this.controller = controller;
        this.tuner = new PIDController(this.controller.getP(), this.controller.getI(), this.controller.getD());
        this.tab = Shuffleboard.getTab(tabName);
        this.tab.add(widgetName, tuner).withPosition(column, 0).withSize(2, 2);
        this.tab.add("Apply " + widgetName + " values", new ApplyValues(this)).withPosition(column, 2).withSize(2, 1);
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


    public class ApplyValues extends CommandBase {
        SparkMaxTuner tuner;
    
        public ApplyValues(SparkMaxTuner tuner) {
            this.tuner = tuner;
        }
    
        
        @Override
        public void initialize() {
            
        }
    
        @Override
        public void execute() {
            tuner.applyTunerValues();
        }
    
        @Override
        public boolean isFinished() {
            return true;
        }
    
        @Override
        public void end(boolean interrupted) {
            
        }
    }



}
