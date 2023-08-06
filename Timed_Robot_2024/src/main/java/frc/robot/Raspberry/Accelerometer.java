package frc.robot.Raspberry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Accelerometer {
    //In the order Position, Velocity, Acceleration
    private double[] x;
    private double[] y;
    private double[] z;
    private DoubleArrayPublisher xPub;
    private DoubleArrayPublisher yPub;
    private DoubleArrayPublisher zPub;

    private double i;
    private double j;
    private double k;
    private double w;
    private DoubleArrayPublisher quaternionPub;

    private AHRS myAccelerometer;

    public Accelerometer(Port aPort) {
        myAccelerometer = new AHRS(aPort);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        myAccelerometer.zeroYaw();
        myAccelerometer.resetDisplacement();

        x = new double[3];
        y = new double[3];
        z = new double[3];

        i = 1.0;
        j = 0.0;
        k = 0.0;
        w = 0.0;

        xPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/x").publish();
        yPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/y").publish();
        zPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/z").publish();
        quaternionPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/quaternion").publish();
    }

    public Accelerometer(Port aPort, byte updateRate) {
        myAccelerometer = new AHRS(aPort, updateRate);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        myAccelerometer.zeroYaw();
        myAccelerometer.resetDisplacement();

        x = new double[3];
        y = new double[3];
        z = new double[3];

        i = 1.0;
        j = 0.0;
        k = 0.0;
        w = 0.0;

        xPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/x").publish();
        yPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/y").publish();
        zPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/z").publish();
        quaternionPub = inst.getDoubleArrayTopic("/raspberry/accelerometer/quaternion").publish();
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

        xPub.set(x);
        SmartDashboard.putNumber("X Displacement: ", x[0]);
        SmartDashboard.putNumber("X Velocity: ", x[1]);
        SmartDashboard.putNumber("X Acceleration: ", x[2]);

        yPub.set(y);
        SmartDashboard.putNumber("Y Displacement: ", y[0]);
        SmartDashboard.putNumber("Y Velocity: ", y[1]);
        SmartDashboard.putNumber("Y Acceleration: ", y[2]);

        zPub.set(z);
        SmartDashboard.putNumber("Z Displacement: ", z[0]);
        SmartDashboard.putNumber("Z Velocity: ", z[1]);
        SmartDashboard.putNumber("Z Acceleration: ", z[2]);

        quaternionPub.set(new double[]{i, j, k, w});
        SmartDashboard.putNumber("Quaternion I: ", i);
        SmartDashboard.putNumber("Quaternion J: ", j);
        SmartDashboard.putNumber("Quaternion K: ", k);
        SmartDashboard.putNumber("Quaternion W: ", w);
    }

    public void close() {
        xPub.close();
        yPub.close();
        zPub.close();
        quaternionPub.close();
        myAccelerometer.close();
    }
}
