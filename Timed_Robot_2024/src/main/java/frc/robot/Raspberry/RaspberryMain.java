package frc.robot.Raspberry;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Constants;

public class RaspberryMain {
    static RaspberryMain currentRaspberryMain;
    Accelerometer myAccelerometer;
    CameraControl myCameras;
    public static void main(String[] args) {
        while(true) {
            if(SmartDashboard.getString("Raspberry Instructions", "").equals("Start")) {
                currentRaspberryMain = new RaspberryMain();
            } else if(SmartDashboard.getString("Raspberry Instructions", "").equals("Update")) {
                currentRaspberryMain.update();
            } else if(SmartDashboard.getString("Raspberry Instructions", "").equals("Close")) {
                currentRaspberryMain.close();
            }

            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public RaspberryMain() {
        myAccelerometer = new Accelerometer(I2C.Port.kMXP);
        myCameras = new CameraControl(CameraServer.startAutomaticCapture("front", 0), 
        CameraServer.startAutomaticCapture("left", 1), CameraServer.startAutomaticCapture("right", 2), 
        CameraServer.startAutomaticCapture("back", 3));

        myAccelerometer.start();
        myCameras.setResolutionAll(Constants.allCamerasResolutionWidth, Constants.allCamerasResolutionHeight);
        myCameras.setFPSAll(Constants.allCamerasFPS);
    }

    public void update() {
        myAccelerometer.update();
        myCameras.update();
    }

    public void close() {
        myAccelerometer.close();
        myCameras.close();
    }
}
