package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TurnToAngle extends PIDCommand {

    DrivetrainSubsystem driveSystem;

    public TurnToAngle(double target) {
      super(new PIDController(Constants.TurnToAngle.P, Constants.TurnToAngle.I, Constants.TurnToAngle.D), DrivetrainSubsystem.getInstance()::getYawAsDegrees, target, output -> useOutput(output), DrivetrainSubsystem.getInstance());
      getController().enableContinuousInput(-180, 180);
      getController().setTolerance(Constants.TurnToAngle.PositionTolerance, Constants.TurnToAngle.VelocityTolerance);
    }
    
    @Override
    public void initialize() {}
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

   
    public static void useOutput(double value) {
      DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
      // get current speeds of the drive to avoid changing translation
      ChassisSpeeds speeds = drive.getChassisSpeeds();
      // change the rotation speed
      speeds.omegaRadiansPerSecond = Rotation2d.fromDegrees(value).getRadians();
      DrivetrainSubsystem.getInstance().drive(speeds);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint();
    }    
}
