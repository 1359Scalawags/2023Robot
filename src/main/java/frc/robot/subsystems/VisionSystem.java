package frc.robot.subsystems;


    import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//positive x value, right negative, left
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSystem extends SubsystemBase {

    public enum LimelightModes {
        vision,
        driver
    }

    public enum USBCameras {
        TopCamera,
        BottomCamera
    }

    // variables for USB Cams
    private UsbCamera camera1;
    //private UsbCamera camera2;
    private VideoSink server;

    // variables for Limelight
    double x, y, area;

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = getLimelightEntry("tx");
    NetworkTableEntry ty = getLimelightEntry("ty");
    NetworkTableEntry ta = getLimelightEntry("ta");


    public VisionSystem() {
        // limelight initialization
        setCamMode(LimelightModes.vision);

        try {
            // USB Camera initialization
            if(Robot.isSimulation()) {
                camera1 = CameraServer.startAutomaticCapture(4);
            } else {
                camera1 = CameraServer.startAutomaticCapture(0);
            }

            camera1.setResolution(Constants.DisplaySystem.CAM_WIDTH, Constants.DisplaySystem.CAM_HEIGHT);
            camera1.setFPS(Constants.DisplaySystem.CAM_FPS);
            camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            //System.out.println(camera1.getConfigJson());
        } catch (Exception e) {
            System.out.println("Vision system could not capture camera!" + e.getMessage());
        }

        // try {
        //     camera2 = CameraServer.startAutomaticCapture(1);
        //     camera2.setResolution(Constants.DisplaySystem.CAM_WIDTH, Constants.DisplaySystem.CAM_HEIGHT);
        //     camera2.setFPS(Constants.DisplaySystem.CAM_FPS);
        //     camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        // } catch (Exception e) {
        //     camera2 = null;
        // }

        //NOTE: this should probably use the "addServer()" function for multiple cameras
        server = CameraServer.getServer();

        server.setSource(camera1);
        // if (camera1 != null) {
        //     server.setSource(camera1);
        // } else if (camera2 != null) {
        //     server.setSource(camera2);
        // }

    }

    public static void setCamMode(LimelightModes mode) {
        getLimelightEntry("camMode").setNumber(mode.ordinal());
    }

    public void setUSBCamera(USBCameras camera) {
    //     if (camera == USBCameras.BottomCamera) {
    //         server.setSource(camera2);
    //     } else if (camera == USBCameras.TopCamera) {
    //         server.setSource(camera1);
    //     } else {
    //         server.setSource(null);
    //     }
    }

    @Override
    public void periodic() {
        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    private static NetworkTableEntry getLimelightEntry(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault().getTable("limelight");
        }
        return table.getEntry(key);
    }

    @Override
    public void simulationPeriodic() {

    }

    public Double getTargetX() {
        return tx.getDouble(0.0);
    }

    public Double getTargetY() {
        return ty.getDouble(0.0);
    }

    public Double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public UsbCamera getCamera1() {
        if(camera1.isValid()) {
            return camera1;            
        } else {
            return null;
        }

    }

}