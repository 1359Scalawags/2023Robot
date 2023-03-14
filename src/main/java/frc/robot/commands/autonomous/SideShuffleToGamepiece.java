package frc.robot.commands.autonomous;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
public class SideShuffleToGamepiece extends CommandBase  {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final VisionSystem m_Vision;
  private final DrivetrainSubsystem m_drivetrainSubsystem; 

  private  ChassisSpeeds m_ChassisSpeeds;
  SlewRateLimiter limit = new SlewRateLimiter(0);
  /**
   * Creates a new ExampleCommand.
   *
   * @param vision The subsystem used by this command.
   */
  public SideShuffleToGamepiece(VisionSystem vision,DrivetrainSubsystem drive, ChassisSpeeds chassis) {
    m_Vision = vision;
    m_ChassisSpeeds = chassis;
    m_drivetrainSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limit.reset(0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, limit.calculate(m_Vision.getTargetX()*0.01), 0));
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Vision.doesTargetExist() == 0){
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
      return true;
   }else{
      if ( Constants.Autonomous.degreeOffset < m_Vision.getTargetX() && m_Vision.getTargetX() < -Constants.Autonomous.degreeOffset){
        return true;
      }
      return false;
   }
  }
}
