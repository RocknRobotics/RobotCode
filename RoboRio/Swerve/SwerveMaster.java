package frc.robot.RoboRio.Swerve;

import frc.robot.RoboRio.Constants;
import frc.robot.RoboRio.Constants.talonConstants.*;

import java.util.EnumSet;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PS4Controller;

public class SwerveMaster {
    private SwerveModule leftUpModule;
    private SwerveModule leftDownModule;
    private SwerveModule rightUpModule;
    private SwerveModule rightDownModule;

    private AHRS accelerometer;

    @SuppressWarnings("unused")
    private NetworkTableListener outputUpdateListener;
    private BooleanEntry inputUpdateEntry;
    private DoubleArrayPublisher controllerInputsPublisher;
    private DoubleArrayPublisher driveVelocitiesPublisher;
    private DoubleArrayPublisher turnPositionsPublisher;
    private DoublePublisher reducedAnglePublisher;
    private BooleanEntry outputUpdateEntry;
    private DoubleArraySubscriber driveSetSubscriber;
    private DoubleArraySubscriber turnSetSubscriber;
    private BooleanPublisher resetOdometerPublisher;
    private DoubleArrayPublisher resetOdometerCurrentPosePublisher;

    private NetworkTableInstance inst;

    public SwerveMaster() {
        inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        
        leftUpModule = new SwerveModule(driveConstants.leftUpID, turnConstants.leftUpID, 
        driveConstants.leftUpInvert, turnConstants.leftUpInvert);
        leftDownModule = new SwerveModule(driveConstants.leftDownID, turnConstants.leftDownID, 
        driveConstants.leftDownInvert, turnConstants.leftDownInvert);
        rightUpModule = new SwerveModule(driveConstants.rightUpID, turnConstants.rightUpID, 
        driveConstants.rightUpInvert, turnConstants.rightUpInvert);
        rightDownModule = new SwerveModule(driveConstants.rightDownID, turnConstants.rightDownID, 
        driveConstants.rightDownInvert, turnConstants.rightDownInvert);

        accelerometer = new AHRS(Port.kMXP, Constants.accelerometerUpdateFrequency);
        accelerometer.reset();

        inputUpdateEntry = inst.getBooleanTopic("/laptop/swerve/update/input").getEntry(false);
        controllerInputsPublisher = inst.getDoubleArrayTopic("/laptop/swerve/controller/inputs").publish();
        driveVelocitiesPublisher = inst.getDoubleArrayTopic("/laptop/swerve/drive/velocities").publish();
        turnPositionsPublisher = inst.getDoubleArrayTopic("/laptop/swerve/turn/positions").publish();
        reducedAnglePublisher = inst.getDoubleTopic("/laptop/swerve/accelerometer/reducedAngle").publish();
        outputUpdateEntry = inst.getBooleanTopic("/laptop/swerve/update/output").getEntry(false);
        driveSetSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/drive/set").subscribe(new double[]{0d, 0d, 0d, 0d});
        turnSetSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/turn/set").subscribe(new double[]{0d, 0d, 0d, 0d});
        resetOdometerPublisher = inst.getBooleanTopic("/laptop/swerve/odometer/reset").publish();
        resetOdometerCurrentPosePublisher = inst.getDoubleArrayTopic("/laptop/swerve/odometer/currentPose").publish();

        outputUpdateListener = NetworkTableListener.createListener(inst.getBooleanTopic("/laptop/swerve/update/output"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                if(outputUpdateEntry.get()) {
                    set(driveSetSubscriber.get(), turnSetSubscriber.get());
                    outputUpdateEntry.set(false);

                    inst.flush();
                }
            }
        });
    }

    public void update(PS4Controller controller) {
        while(!inputUpdateEntry.get()) {
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }

        controllerInputsPublisher.set(new double[]{controller.getLeftX(), controller.getLeftY(), controller.getRightX()});
        driveVelocitiesPublisher.set(new double[]{leftUpModule.getDriveVelocity(), leftDownModule.getDriveVelocity(), 
        rightUpModule.getDriveVelocity(), rightDownModule.getDriveVelocity()});
        turnPositionsPublisher.set(new double[]{leftUpModule.getTurnPosition(), leftDownModule.getTurnPosition(), 
        rightUpModule.getTurnPosition(), rightDownModule.getTurnPosition()});
        reducedAnglePublisher.set(this.getReducedAngle());
        outputUpdateEntry.set(false);

        inst.flush();
    }

    public void set(double[] driveSets, double[] turnSets) {
        leftUpModule.set(driveSets[0], turnSets[0]);
        leftDownModule.set(driveSets[1], turnSets[1]);
        rightUpModule.set(driveSets[2], turnSets[2]);
        rightDownModule.set(driveSets[3], turnSets[3]);
    }

    public void resetAccelerometer() {
        accelerometer.reset();
    }

    public boolean accelerometerIsCalibrating() {
        return accelerometer.isCalibrating();
    }

    public double getReducedAngle() {
        return Math.IEEEremainder(accelerometer.getAngle(), 360d);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getReducedAngle());
    }

    public void resetOdometer(double currX, double currY, double currAngle) {
        resetOdometerCurrentPosePublisher.set(new double[]{currX, currY, currAngle});
        resetOdometerPublisher.set(true);

        inst.flush();
    }
}
