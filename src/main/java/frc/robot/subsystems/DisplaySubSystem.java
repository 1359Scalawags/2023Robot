package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
//TODO: Figure out the correct way to do this system
import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.autonomous.BlueStationONE;
import frc.robot.commands.autonomous.BlueStationTHREE;
import frc.robot.commands.autonomous.BlueStationTWO;
import frc.robot.commands.autonomous.LoadGamepiece;
import frc.robot.commands.autonomous.RedStationONE;
import frc.robot.commands.autonomous.RedStationTHREE;
import frc.robot.commands.autonomous.RedStationTWO;
import frc.robot.commands.autonomous.TestAutoMovment;

public class DisplaySubSystem extends SubsystemBase {
    PowerDistribution m_pdh = new PowerDistribution(Constants.DisplaySystem.PDHCANID, ModuleType.kRev);
    private ShuffleboardTab mainTab = Shuffleboard.getTab("Match Tab");

    private GenericEntry timeEntry; 
    private GenericEntry batteryVoltage; 
    private GenericEntry driveModeEntry;
    private HttpCamera camera = new HttpCamera("Camera view", "http://10.13.59.11:5800");
    private DrivetrainSubsystem driveSystem;
    SendableChooser<Command> chooser = new SendableChooser<>();
    
    // private NetworkTableEntry climbLockEntry;
    // private ComplexWidget cameraView;
    // private ClimbSystem climbSystem;

    public DisplaySubSystem(VisionSystem vision, DrivetrainSubsystem driveSystem, ArmSubsystem armSystem, GrabberSubsystem grabberSystem) {
        CameraServer.startAutomaticCapture(camera);
        this.driveSystem = driveSystem;
        // climbSystem = climber;
        // Shuffleboard.selectTab("Match Tab");
        timeEntry = mainTab
                .add("Match Time", 0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 135))
                .withSize(3, 3)
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
                .withPosition(0, 1)
                .getEntry();

        // if(vision.getCamera1() != null) {
        // this.mainTab
        //         .add("Camera", vision.getCamera1())
        //         .withWidget(BuiltInWidgets.kCameraStream)
        //         .withSize(4, 4)
        //         .withPosition(3, 0);

        this.mainTab.add(camera)
                    .withSize(4, 4);

        driveModeEntry = mainTab
                .add("Drive Mode", driveSystem.getDriveMode())
                .withSize(2, 1)
                .getEntry();
        // }
        
        //chooser.addOption("Test Movment", new TestAutoMovment(driveSystem, true));
        //chooser.addOption("Test Movment", new TestAutoMovment(driveSystem, false));
        chooser.addOption("BlueStation1 ChargeStation", new BlueStationONE(driveSystem, armSystem, grabberSystem, true));
        chooser.addOption("BlueStation1", new BlueStationONE(driveSystem, armSystem, grabberSystem, false));
        chooser.addOption("BlueStation2 ChargeStation", new BlueStationTWO(driveSystem, armSystem, grabberSystem, true));
        chooser.addOption("BlueStation2", new BlueStationTWO(driveSystem, armSystem, grabberSystem, false));
        chooser.addOption("BlueStation3 ChargeStation", new BlueStationTHREE(driveSystem, armSystem, grabberSystem, true));
        chooser.addOption("BlueStation3", new BlueStationTHREE(driveSystem, armSystem, grabberSystem, false));
        chooser.addOption("RedStation1 ChargeStation", new RedStationONE(driveSystem, armSystem, grabberSystem, true));
        chooser.addOption("RedStation1", new RedStationONE(driveSystem, armSystem, grabberSystem, false));
        chooser.addOption("RedStation2 ChargeStation", new RedStationTWO(driveSystem, armSystem, grabberSystem, true));
        chooser.addOption("RedStation2", new RedStationTWO(driveSystem, armSystem, grabberSystem, false));
        chooser.addOption("RedStation3 ChargeStation", new RedStationTHREE(driveSystem, armSystem, grabberSystem, true));
        chooser.addOption("RedStation3", new RedStationTHREE(driveSystem, armSystem, grabberSystem, false));
        chooser.addOption("Null", new InstantCommand());
        //chooser.addOption("Test Loading piece", new LoadGamepiece(armSystem, grabberSystem));
        mainTab.add(chooser);
    }

    public SendableChooser<Command> getAutonomoChooser() {
        return chooser;
    }
    private int counter = 0;

    @Override
    public void periodic() {
        if (counter > 5) {
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