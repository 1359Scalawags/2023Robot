// package frc.robot.subsystems;

// import java.util.Map;
//TODO: Figure out the correct way to do this system
// import edu.wpi.first.networktables.GenericEntry;
// //import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// //import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class DisplaySystem extends SubsystemBase {
//     PowerDistribution m_pdh = new PowerDistribution(Constants.DisplaySystem.PDHCANID, ModuleType.kRev);

//     private ShuffleboardTab mainTab = Shuffleboard.getTab("Match Tab");
//     private GenericEntry timeEntry; // ?Changed NetworkTableEntry to GenericEntry?
//     private GenericEntry batteryVoltage; // ?Changed NetworkTableEntry to GenericEntry?
//     // private NetworkTableEntry climbLockEntry;
//     // private ComplexWidget cameraView;
//     // private ClimbSystem climbSystem;

//     public DisplaySystem(VisionSystem vision) {
//         // climbSystem = climber;
//         Shuffleboard.selectTab("Match Tab");
//         timeEntry = mainTab
//                 .add("Match Time", 0)
//                 .withWidget(BuiltInWidgets.kDial)
//                 .withProperties(Map.of("min", 0, "max", 135))
//                 .withSize(3, 3)
//                 .withPosition(7, 0)
//                 .getEntry();

//         // climbLockEntry = mainTab
//         // .add("Climber Status", true)
//         // .withWidget(BuiltInWidgets.kBooleanBox)
//         // .withSize(3,1)
//         // .withProperties(Map.of("Color when true", "green", "Color when false",
//         // "maroon"))
//         // .withPosition(0, 0)
//         // .getEntry();

//         batteryVoltage = mainTab
//                 .add("Battery Voltage", 0)
//                 .withWidget(BuiltInWidgets.kNumberBar)
//                 .withProperties(Map.of("min", 0, "max", 16))
//                 .withSize(3, 1)
//                 .withPosition(0, 1)
//                 .getEntry();

//         // if(vision.getCamera1() != null) {
//         this.mainTab
//                 .add("Camera", vision.getCamera1())
//                 .withWidget(BuiltInWidgets.kCameraStream)
//                 .withSize(4, 4)
//                 .withPosition(3, 0);
//         // }

//     }

//     private int counter = 0;

//     @Override
//     public void periodic() {
//         if (counter > 5) {
//             if (timeEntry != null) {
//                 timeEntry.setDouble(DriverStation.getMatchTime());
//             }
//             // if(climbLockEntry != null) {
//             // climbLockEntry.setBoolean(!climbSystem.getMasterLocked());
//             // }
//             if (batteryVoltage != null) {
//                 batteryVoltage.setDouble(m_pdh.getVoltage());
//             }
//             counter = 0;
//         }

//         counter++;
//     }

// }