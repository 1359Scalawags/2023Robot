package frc.robot.commands;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WheelPositions;
import frc.robot.extensions.Utilities;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnWheelToAngleCommand extends CommandBase  {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private WheelPositions m_wheelPosition;
    private double m_speed;
    private double m_angle;
    private SwerveModule m_module;

    /***
     * 
     * @param drivetrainSubsystem
     * @param wheel which wheel to use
     * @param speed speed from 0 to 1
     * @param angle angle in degrees
     */
    public TurnWheelToAngleCommand(DrivetrainSubsystem drivetrainSubsystem, WheelPositions wheel, double speed,
                               double angle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_angle = angle;
        this.m_speed = speed;
        this.m_wheelPosition = wheel;
        this.m_module = m_drivetrainSubsystem.getModule(m_wheelPosition);
        addRequirements(drivetrainSubsystem);
    }

    @Override 
    public void initialize() {
        m_drivetrainSubsystem.setModule(m_wheelPosition, m_speed, m_angle);     
        m_module.getDriveVelocity(); 
    }

    @Override
    public void execute() {
        
    }

    @Override public boolean isFinished() {
        if(Utilities.IsCloseTo(m_module.getSteerAngle(), Math.toRadians(m_angle), Math.toRadians(3.0))) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
