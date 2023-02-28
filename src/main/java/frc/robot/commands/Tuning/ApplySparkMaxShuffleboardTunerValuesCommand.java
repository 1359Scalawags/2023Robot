package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extensions.SparkMaxShufflboardPIDTuner;

/*
 * A command for applying values changed by the SparkmaxPIDTuner
 */
public class ApplySparkMaxShuffleboardTunerValuesCommand extends CommandBase {
    SparkMaxShufflboardPIDTuner tuner;

    public ApplySparkMaxShuffleboardTunerValuesCommand(SparkMaxShufflboardPIDTuner tuner) {
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
