package frc.robot.Raspberry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Accelerometer {
    //In the order Position, Velocity, Acceleration
    private static double[] x;
    private static double[] y;
    private static double[] z;

    private static double i;
    private static double j;
    private static double k;
    private static double w;

    AHRS myAccelerometer;

    public Accelerometer(Port aPort) {
        myAccelerometer = new AHRS(aPort);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);
    }
    public Accelerometer(Port aPort, byte updateRate) {
        myAccelerometer = new AHRS(aPort, updateRate);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);
    }

    public void start() {
        myAccelerometer.zeroYaw();
        myAccelerometer.resetDisplacement();

        x = new double[3];
        y = new double[3];
        z = new double[3];

        i = 1.0;
        j = 0.0;
        k = 0.0;
        w = 0.0;
    }

    public void update() {
        x[0] = myAccelerometer.getDisplacementX();
        x[1] = myAccelerometer.getVelocityX();
        x[2] = myAccelerometer.getWorldLinearAccelX();

        y[0] = myAccelerometer.getDisplacementY();
        y[1] = myAccelerometer.getVelocityY();
        y[2] = myAccelerometer.getWorldLinearAccelY();

        z[0] = myAccelerometer.getDisplacementZ();
        z[1] = myAccelerometer.getVelocityZ();
        z[2] = myAccelerometer.getWorldLinearAccelZ();

        i = myAccelerometer.getQuaternionX();
        j = myAccelerometer.getQuaternionY();
        k = myAccelerometer.getQuaternionZ();
        w = myAccelerometer.getQuaternionW();

        SmartDashboard.putNumber("X Displacement: ", x[0]);
        SmartDashboard.putNumber("X Velocity: ", x[1]);
        SmartDashboard.putNumber("X Acceleration: ", x[2]);

        SmartDashboard.putNumber("Y Displacement: ", y[0]);
        SmartDashboard.putNumber("Y Velocity: ", y[1]);
        SmartDashboard.putNumber("Y Acceleration: ", y[2]);

        SmartDashboard.putNumber("Z Displacement: ", z[0]);
        SmartDashboard.putNumber("Z Velocity: ", z[1]);
        SmartDashboard.putNumber("Z Acceleration: ", z[2]);

        SmartDashboard.putNumber("Quaternion I: ", i);
        SmartDashboard.putNumber("Quaternion J: ", j);
        SmartDashboard.putNumber("Quaternion K: ", k);
        SmartDashboard.putNumber("Quaternion W: ", w);
    }

    public void close() {
        myAccelerometer.close();
    }
}
