package frc.robot.subsystems;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwitchPipeline;
import frc.robot.commands.autonomous.BlueStationONE;
import frc.robot.commands.autonomous.BlueStationTHREE;
import frc.robot.commands.autonomous.BlueStationTWO;
import frc.robot.commands.autonomous.LoadGamepieceOnHighLevel;
import frc.robot.commands.autonomous.MoveBackwards;
import frc.robot.commands.autonomous.OnlyMoving;
import frc.robot.commands.autonomous.RedStationONE;
import frc.robot.commands.autonomous.RedStationTHREE;
import frc.robot.commands.autonomous.RedStationTWO;
import frc.robot.commands.autonomous.StandardAuto;
import frc.robot.commands.autonomous.TestingPathplanner;

public class DisplaySubSystem extends SubsystemBase {
    PowerDistribution m_pdh = new PowerDistribution(Constants.DisplaySystem.PDHCANID, ModuleType.kRev);
    private ShuffleboardTab mainTab = Shuffleboard.getTab("Match Tab");

    private GenericEntry timeEntry; 
    private GenericEntry batteryVoltage; 
    private GenericEntry driveModeEntry;
    private HttpCamera camera = new HttpCamera("limelight", "http://10.13.59.11:5800/stream.mjpeg", HttpCameraKind.kMJPGStreamer);
    private DrivetrainSubsystem driveSystem;
    private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();
    
    // private NetworkTableEntry climbLockEntry;
    // private ComplexWidget cameraView;
    // private ClimbSystem climbSystem;

    // private final SendableChooser<Command> pipeLine =  new SendableChooser<>();



    public DisplaySubSystem(VisionSystem vision, DrivetrainSubsystem driveSystem, ArmSubsystem armSystem, GrabberSubsystem grabberSystem) {
        CameraServer.startAutomaticCapture(camera);
        this.driveSystem = driveSystem;
        // climbSystem = climber;
        // Shuffleboard.selectTab("Match Tab");
        timeEntry = mainTab
                .add("Match Time", 0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 135))
                .withSize(3, 2)
                .withPosition(7, 0) 
                .getEntry();

        // climbLockEntry = mainTab
        // .add("Climber Status", true)
        // .withWidget(BuiltInWidgets.kBooleanBox)
        // .withSize(3,1)
        // .withProperties(Map.of("Color when true", "green", "Color when false",
        // "maroon"))
        // .withPosition(0, 0)
        // .getEntry();

        batteryVoltage = mainTab
                .add("Battery Voltage", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 16))
                .withSize(3, 1)
                .withPosition(7, 2)
                .getEntry();

        // if(vision.getCamera1() != null) {
        // this.mainTab
        //         .add("Camera", vision.getCamera1())
        //         .withWidget(BuiltInWidgets.kCameraStream)
        //         .withSize(4, 4)
        //         .withPosition(3, 0);

        mainTab.add(camera)
                    .withSize(5, 4)
                    .withPosition(2, 0);

        driveModeEntry = mainTab
                .add("Drive Mode", driveSystem.getDriveMode())
                .withSize(2, 1)
                .withPosition(0, 0)
                .getEntry();
        // }
        
        //chooser.addOption("Test Movment", new TestAutoMovment(driveSystem, true));
        //chooser.addOption("Test Movment", new TestAutoMovment(driveSystem, false));
        // autonomousChooser.addOption("BlueStation1 ChargeStation", new BlueStationONE(driveSystem, armSystem, grabberSystem, true));
        //chooser.addOption("BlueStation1", new BlueStationONE(driveSystem, armSystem, grabberSystem, false));
        // autonomousChooser.addOption("BlueStation2 ChargeStation", new BlueStationTWO(driveSystem, armSystem, grabberSystem, true));
        //chooser.addOption("BlueStation2", new BlueStationTWO(driveSystem, armSystem, grabberSystem, false));
        // autonomousChooser.addOption("BlueStation3 ChargeStation", new BlueStationTHREE(driveSystem, armSystem, grabberSystem, true));
        //chooser.addOption("BlueStation3", new BlueStationTHREE(driveSystem, armSystem, grabberSystem, false));
        // autonomousChooser.addOption("RedStation3 ChargeStation", new RedStationTHREE(driveSystem, armSystem, grabberSystem, true));
        //chooser.addOption("RedStation3", new RedStationTHREE(driveSystem, armSystem, grabberSystem, false));
        // autonomousChooser.addOption("RedStation2 ChargeStation", new RedStationTWO(driveSystem, armSystem, grabberSystem, true));
        //chooser.addOption("RedStation2", new RedStationTWO(driveSystem, armSystem, grabberSystem, false));
        // autonomousChooser.addOption("RedStation1 ChargeStation", new RedStationONE(driveSystem, armSystem, grabberSystem, true));
        //chooser.addOption("RedStation1", new RedStationONE(driveSystem, armSystem, grabberSystem, false));
        autonomousChooser.addOption("Full standard auto", new StandardAuto(driveSystem, armSystem, grabberSystem, false));
        autonomousChooser.setDefaultOption("Only loading", new LoadGamepieceOnHighLevel(armSystem, grabberSystem));
        autonomousChooser.addOption("Only moving", new OnlyMoving(driveSystem));
        autonomousChooser.addOption("Straight path (Test)", driveSystem.followTrajectoryCommand(driveSystem.getStraightPath(), true));
        autonomousChooser.addOption("Curvy path (Test)", driveSystem.followTrajectoryCommand(driveSystem.getCurvyPath(), true));
        autonomousChooser.addOption("Blue1", new BlueStationONE(driveSystem, armSystem, grabberSystem));
        autonomousChooser.addOption("Blue3", new BlueStationTHREE(driveSystem, armSystem, grabberSystem));
        autonomousChooser.addOption("Red1", new RedStationONE(driveSystem, armSystem, grabberSystem));
        autonomousChooser.addOption("Red3", new RedStationTHREE(driveSystem, armSystem, grabberSystem));
        autonomousChooser.addOption("Test Pathplanner", new TestingPathplanner(driveSystem, armSystem, grabberSystem));

        // chooser.addOption("Test Auto movement", new TestAutoMovment(driveSystem));
        autonomousChooser.addOption("Not moving", new InstantCommand());
        
        //chooser.addOption("Test Loading piece", new LoadGamepiece(armSystem, grabberSystem));
        mainTab.add(autonomousChooser)
               .withSize(2, 1)
               .withPosition(0, 1);

        //chooser.addOption("Test Loading piece", new LoadGamepiece(armSystem, grabberSystem));
        // mainTab.add(chooser);
        //pipeLine.addOption(vision, kCubeWhiteLight);
        
        // SwitchPipeline pipeLineCommand = new SwitchPipeline(vision, null);
        // //pipeLine.addOption("Default", pipeLineCommand);
        // pipeLine.setDefaultOption("Default", pipeLineCommand);
        
        // pipeLineCommand.set(SwitchPipeline.pipeIndex.CubeWhiteLight);
        // pipeLine.addOption("CubeWhiteLight", pipeLineCommand);
        
        // pipeLineCommand.set(SwitchPipeline.pipeIndex.ConeWhiteLight);
        // pipeLine.addOption("ConeWhiteLight", pipeLineCommand);
        
        // pipeLineCommand.set(SwitchPipeline.pipeIndex.CubeYellowLight);
        // pipeLine.addOption("CubeYellowLight", pipeLineCommand);
        
        // pipeLineCommand.set(SwitchPipeline.pipeIndex.ConeYellowLight);
        // pipeLine.addOption("ConeYellowLight", pipeLineCommand);
        
        
        // mainTab.add(pipeLine)
        //        .withSize(2, 1)
        //        .withPosition(0, 2);

        
    }

    

    public SendableChooser<Command> getAutonomousChooser() {
        return autonomousChooser;
    }
    public ShuffleboardTab getMainTab() {
        return mainTab;
    }
    
    private int counter = 0;

    @Override
    public void periodic() {
        if (counter > 10) {
            if (timeEntry != null) {
                timeEntry.setDouble(DriverStation.getMatchTime());
            }
            // if(climbLockEntry != null) {
            // climbLockEntry.setBoolean(!climbSystem.getMasterLocked());
            // }
            if (batteryVoltage != null) {
                batteryVoltage.setDouble(m_pdh.getVoltage());
            }
            if (!driveSystem.getDriveMode().equals(driveModeEntry.getString("None"))) {
                driveModeEntry.setString(driveSystem.getDriveMode());
            }
            counter = 0;
        }

        counter++;
    }

}
