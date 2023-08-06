package frc.robot.Laptop;

import java.util.EnumSet;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InformationComparer {
    DoubleArraySubscriber xSub;
    DoubleArraySubscriber ySub;
    DoubleArraySubscriber zSub;
    DoubleArraySubscriber quaternionSub;
    NetworkTableListener accelerometerListener;
    BooleanEntry usedAccelerometer;
    DoubleSubscriber leftUpEncoder;
    DoubleSubscriber rightUpEncoder;
    DoubleSubscriber rightDownEncoder;
    DoubleSubscriber leftDownEncoder;
    NetworkTableListener encoderListener;
    BooleanEntry usedEncoder;
    DoubleArraySubscriber frontXSub;
    DoubleArraySubscriber frontYSub;
    NetworkTableListener frontListener;
    BooleanEntry usedFront;
    DoubleArraySubscriber leftXSub;
    DoubleArraySubscriber leftYSub;
    NetworkTableListener leftListener;
    BooleanEntry usedLeft;
    DoubleArraySubscriber rightXSub;
    DoubleArraySubscriber rightYSub;
    NetworkTableListener rightListener;
    BooleanEntry usedRight;
    DoubleArraySubscriber backXSub;
    DoubleArraySubscriber backYSub;
    NetworkTableListener backListener;
    BooleanEntry usedBack;

    public InformationComparer() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        xSub = inst.getDoubleArrayTopic("/raspberry/accelerometer/x").subscribe(new double[]{0.0, 0.0, 0.0});
        ySub = inst.getDoubleArrayTopic("/raspberry/accelerometer/y").subscribe(new double[]{0.0, 0.0, 0.0});
        zSub = inst.getDoubleArrayTopic("/raspberry/accelerometer/z").subscribe(new double[]{0.0, 0.0, 0.0});
        quaternionSub = inst.getDoubleArrayTopic("/raspberry/accelerometer/quaternion").subscribe(new double[]{1.0, 0.0, 0.0, 0.0});
        accelerometerListener = NetworkTableListener.createListener(inst.getTopic("/raspberry/accelerometer/quaternion"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                usedAccelerometer.set(false);
                inst.flush();
            }
        });

        leftUpEncoder = inst.getDoubleTopic("/robot/encoder/leftUp").subscribe(0.0);
        rightUpEncoder = inst.getDoubleTopic("/robot/encoder/rightUp").subscribe(0.0);
        rightDownEncoder = inst.getDoubleTopic("/robot/encoder/rightDown").subscribe(0.0);
        leftDownEncoder = inst.getDoubleTopic("/robot/encoder/leftDown").subscribe(0.0);
        encoderListener = NetworkTableListener.createListener(inst.getTopic("/robot/encoder/leftDown"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                usedEncoder.set(false);
                inst.flush();
            }
        });

        frontXSub = inst.getDoubleArrayTopic("/raspberry/camera/front/x").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        frontYSub = inst.getDoubleArrayTopic("/raspberry/camera/front/y").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        frontListener = NetworkTableListener.createListener(inst.getTopic("/raspberry/camera/front/y"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                usedFront.set(false);
                inst.flush();
            }
        });
        leftXSub = inst.getDoubleArrayTopic("/raspberry/camera/left/x").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        leftYSub = inst.getDoubleArrayTopic("/raspberry/camera/left/y").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        leftListener = NetworkTableListener.createListener(inst.getTopic("/raspberry/camera/left/y"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                usedLeft.set(false);
                inst.flush();
            }
        });
        rightXSub = inst.getDoubleArrayTopic("/raspberry/camera/right/x").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        rightYSub = inst.getDoubleArrayTopic("/raspberry/camera/right/y").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        rightListener = NetworkTableListener.createListener(inst.getTopic("/raspberry/camera/right/y"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                usedRight.set(false);
                inst.flush();
            }
        });
        backXSub = inst.getDoubleArrayTopic("/raspberry/camera/back/x").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        backYSub = inst.getDoubleArrayTopic("/raspberry/camera/back/y").subscribe(new double[]{0.0, 0.0, 0.0, 0.0});
        backListener = NetworkTableListener.createListener(inst.getTopic("/raspberry/camera/back/y"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                usedBack.set(false);
                inst.flush();
            }
        });
    }

    public void close() {
        xSub.close();
        ySub.close();
        zSub.close();
        quaternionSub.close();
        accelerometerListener.close();
        leftUpEncoder.close();
        rightUpEncoder.close();
        rightDownEncoder.close();
        leftDownEncoder.close();
        encoderListener.close();
        frontXSub.close();
        frontYSub.close();
        frontListener.close();
        leftXSub.close();
        leftYSub.close();
        leftListener.close();
        rightXSub.close();
        rightYSub.close();
        rightListener.close();
        backXSub.close();
        backYSub.close();
        backListener.close();
    }
}
