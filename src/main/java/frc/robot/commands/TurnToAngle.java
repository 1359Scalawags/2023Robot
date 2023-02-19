package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {

    DrivetrainSubsystem driveSystem;

    public TurnToAngle(double target) {
      super(new PIDController(0.01, 0.0001, 0.0), DrivetrainSubsystem.getInstance()::getYawAsRadians, target, output -> useOutput(output), DrivetrainSubsystem.getInstance());
      getController().enableContinuousInput(-180, 180);
      getController().setTolerance(1.0, 0.05);
    }
    
    @Override
    public void initialize() {}
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

   
    public static void useOutput(double value) {
      DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
      // get current speeds of the drive to avoid changing translation
      ChassisSpeeds speeds = drive.getSpeeds();
      // change the rotation speed
      speeds.omegaRadiansPerSecond = value;
      DrivetrainSubsystem.getInstance().drive(null);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint();
    }    
}
