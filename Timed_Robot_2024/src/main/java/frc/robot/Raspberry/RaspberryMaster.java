package frc.robot.Raspberry;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;

public class RaspberryMaster {
    Accelerometer myAccelerometer;
    CameraControl myCameraControl;

    public RaspberryMaster() {}

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
