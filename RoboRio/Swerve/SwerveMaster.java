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
    //Listener for when the laptop outputs (inputs for rio) get updated
    private NetworkTableListener outputUpdateListener;
    //Publishes drive controller inputs (left joystick X, left joystick Y, right joystick X)
    private DoubleArrayPublisher controllerInputsPublisher;
    //Publishes current drive velocities (metres/second)
    private DoubleArrayPublisher driveVelocitiesPublisher;
    //Publishes current turn positions (radians)
    private DoubleArrayPublisher turnPositionsPublisher;
    //Publishes reduced angle reported by accelerometer (degrees)
    private DoublePublisher reducedAnglePublisher;
    //Reads when there's new values to set the motors to + tells laptop it's open to accepting new values
    private BooleanEntry outputUpdateEntry;
    //Reads the values to set the drive motors to
    private DoubleArraySubscriber driveSetSubscriber;
    //Reads the values to set the turn motors to
    private DoubleArraySubscriber turnSetSubscriber;
    //Tells the laptop whether or not to reset the odometer with the Pose2d given below
    private BooleanPublisher resetOdometerPublisher;
    //The Pose2d to use when reseting the odometer
    private DoubleArrayPublisher resetOdometerCurrentPosePublisher;

    private NetworkTableInstance myInstance;

    public SwerveMaster(NetworkTableInstance inst) {
        myInstance = inst;

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

        //rio
        controllerInputsPublisher = inst.getDoubleArrayTopic("/rio/swerve/controller/inputs").publish();
        driveVelocitiesPublisher = inst.getDoubleArrayTopic("/rio/swerve/drive/velocities").publish();
        turnPositionsPublisher = inst.getDoubleArrayTopic("/rio/swerve/turn/positions").publish();
        reducedAnglePublisher = inst.getDoubleTopic("/rio/swerve/accelerometer/reducedAngle").publish();
        resetOdometerCurrentPosePublisher = inst.getDoubleArrayTopic("/rio/swerve/odometer/currentPose").publish();
        //Laptop side
        outputUpdateEntry = inst.getBooleanTopic("/laptop/swerve/update/output").getEntry(false);
        driveSetSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/drive/set").subscribe(new double[]{0d, 0d, 0d, 0d});
        turnSetSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/turn/set").subscribe(new double[]{0d, 0d, 0d, 0d});
        resetOdometerPublisher = inst.getBooleanTopic("/laptop/swerve/odometer/reset").publish();

        //Listens on the event the outputs get updated
        outputUpdateListener = NetworkTableListener.createListener(inst.getBooleanTopic("/laptop/swerve/update/output"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                //This way it's not triggered by when it gets set to false
                if(outputUpdateEntry.get()) {
                    set(driveSetSubscriber.get(), turnSetSubscriber.get());
                    outputUpdateEntry.set(false);

                    myInstance.flush();
                }
            }
        });
    }

    public void update(PS4Controller controller, double factor) {
        controllerInputsPublisher.set(new double[]{controller.getLeftX() * factor, controller.getLeftY() * factor, controller.getRightX() * factor});
        driveVelocitiesPublisher.set(new double[]{leftUpModule.getDriveVelocity(), leftDownModule.getDriveVelocity(), 
        rightUpModule.getDriveVelocity(), rightDownModule.getDriveVelocity()});
        turnPositionsPublisher.set(new double[]{leftUpModule.getTurnPosition(), leftDownModule.getTurnPosition(), 
        rightUpModule.getTurnPosition(), rightDownModule.getTurnPosition()});
        reducedAnglePublisher.set(this.getReducedAngle());
        outputUpdateEntry.set(false);

        myInstance.flush();
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
        //You've got to be joking the darn navx is cw positive when we need ccw positive readings
        return Math.IEEEremainder(-accelerometer.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getReducedAngle());
    }

    public void resetOdometer(double currX, double currY, double currAngle) {
        resetOdometerCurrentPosePublisher.set(new double[]{currX, currY, currAngle});
        resetOdometerPublisher.set(true);

        myInstance.flush();
    }
}
