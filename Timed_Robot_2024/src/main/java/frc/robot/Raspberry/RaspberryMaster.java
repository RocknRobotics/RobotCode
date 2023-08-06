package frc.robot.Raspberry;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryMaster {
    Accelerometer myAccelerometer;
    CameraControl myCameraControl;

    public RaspberryMaster() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);
    }

    public void robotInit() {
        myAccelerometer = new Accelerometer(I2C.Port.kMXP);
        myCameraControl = new CameraControl(CameraServer.startAutomaticCapture("front", 0), 
        CameraServer.startAutomaticCapture("left", 1), CameraServer.startAutomaticCapture("right", 2), 
        CameraServer.startAutomaticCapture("back", 3));
    }

    public void robotPeriodic() {

    }

    public void autonomousInit() {

    }

    public void autonomousPeriodic() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {

    }

    public void disabledInit() {

    }

    public void disabledPeriodic() {

    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void close() {
        myAccelerometer.close();
        myCameraControl.close();
    }
}
