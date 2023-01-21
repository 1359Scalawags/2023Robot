package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TurnToAngle extends PIDCommand {

    public TurnToAngle(double target) {
      super(new PIDController(0.01, 0.0001, 0.0), DrivetrainSubsystem.getInstance()::getYaw, target, output -> useOutput(output), DrivetrainSubsystem.getInstance());
      getController().enableContinuousInput(-180, 180);
      getController().setTolerance(1.0, 0.05);
    }
    
    @Override
    public void initialize() {}
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

   
    public static void useOutput(double value) {

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint();
    }    
}
