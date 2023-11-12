package frc.robot.Laptop;

import java.util.EnumSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RoboRio.Constants;
import frc.robot.RoboRio.Constants.talonConstants.driveConstants;
import frc.robot.RoboRio.Constants.talonConstants.turnConstants;

public class SwerveCalculator {
    //Listener for when values are ready to be updated
    @SuppressWarnings("unused")
    private NetworkTableListener inputUpdateListener;
    private BooleanEntry inputUpdateEntry;
    private DoubleArraySubscriber controllerInputsSubscriber;
    private DoubleArraySubscriber driveVelocitiesSubscriber;
    private DoubleArraySubscriber turnPositionsSubscriber;
    private DoubleSubscriber reducedAngleSubscriber;
    private DoubleArrayPublisher driveSetPublisher;
    private DoubleArrayPublisher turnSetPublisher;
    private BooleanEntry outputUpdateEntry;
    private BooleanEntry resetOdometerEntry;
    private DoubleArraySubscriber resetOdometerCurrentPoseSubscriber;

    private SwerveDriveOdometry odometer;

    //Enables continious input I think? I'm writing this using another person's code as a guide---I'll mess around with changing
    //this once we have the swerve drive built
    private PIDController turnPIDController;

    private NetworkTableInstance inst;

    public SwerveCalculator() {
        inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        inputUpdateEntry = inst.getBooleanTopic("/laptop/swerve/update/input").getEntry(false);
        controllerInputsSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/controller/inputs").subscribe(new double[]{0d, 0d, 0d});
        driveVelocitiesSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/drive/velocities").subscribe(new double[]{0d, 0d, 0d});
        turnPositionsSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/turn/positions").subscribe(new double[]{0d, 0d, 0d});
        reducedAngleSubscriber = inst.getDoubleTopic("/laptop/swerve/accelerometer/reducedAngle").subscribe(0d);
        driveSetPublisher = inst.getDoubleArrayTopic("/laptop/swerve/drive/set").publish();
        turnSetPublisher = inst.getDoubleArrayTopic("/laptop/swerve/turn/set").publish();
        outputUpdateEntry = inst.getBooleanTopic("/laptop/swerve/update/output").getEntry(false);
        resetOdometerEntry = inst.getBooleanTopic("/laptop/swerve/odometer/reset").getEntry(false);
        resetOdometerCurrentPoseSubscriber = inst.getDoubleArrayTopic("/laptop/swerve/odometer/reset/currentPose").subscribe(new double[]{0d, 0d, 0d});

        inputUpdateListener = NetworkTableListener.createListener(inst.getBooleanTopic("/laptop/swerve/update/input"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                if(inputUpdateEntry.get()) {
                    if(resetOdometerEntry.get()) {
                        resetOdometer(driveVelocitiesSubscriber.get(), turnPositionsSubscriber.get(), reducedAngleSubscriber.get(), new Pose2d(
                            resetOdometerCurrentPoseSubscriber.get()[0], resetOdometerCurrentPoseSubscriber.get()[1], new Rotation2d(resetOdometerCurrentPoseSubscriber.get()[2])));
                    }

                    update(controllerInputsSubscriber.get(), driveVelocitiesSubscriber.get(), turnPositionsSubscriber.get(), reducedAngleSubscriber.get());

                    inputUpdateEntry.set(false);
                    inst.flush();
                }
            }
        });

        turnPIDController = new PIDController(turnConstants.kP, 0d, 0d);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void update(double[] inputs, double[] velocities, double[] positions, double reducedAngle) {
        //Arrays to be published later
        double driveSets[] = new double[]{0d, 0d, 0d, 0d};
        double turnSets[] = new double[]{0d, 0d, 0d, 0d};
        //Converts PS4 joystick inputs into field-relative speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(inputs[0] * Constants.maxTranslationalSpeed, 
        inputs[1] * Constants.maxTranslationalSpeed, inputs[2] * Constants.maxTranslationalSpeed, Rotation2d.fromDegrees(reducedAngle));
        //Converts those speeds to targetStates since I'm not a monster who puts everything on one line (I had to resist the urge to)
        SwerveModuleState[] targetStates = driveConstants.driveTalonKinematics.toSwerveModuleStates(speeds);

        //Scales the targetStates in case it's not physically possible (for example it's impossible to go the true full move velocity 
        //and true full rotational velocity at the same time)
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, driveConstants.driveTalonKinematics.toChassisSpeeds(
            new SwerveModuleState[]{new SwerveModuleState(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModuleState(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModuleState(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModuleState(velocities[3], Rotation2d.fromRadians(positions[3]))}), 
        driveConstants.maxSpeed, Constants.maxTranslationalSpeed, Constants.maxAngularSpeed);

        //Optimize the states
        for(int i = 0; i < targetStates.length; i++) {
            if(Math.abs(targetStates[i].speedMetersPerSecond) < driveConstants.stopBelowThisVelocity) {
                driveSets[i] = 0d;
                turnSets[i] = 0d;
            } else {
                SwerveModuleState.optimize(targetStates[i], Rotation2d.fromRadians(positions[i]));
                driveSets[i] = targetStates[i].speedMetersPerSecond / (driveConstants.maxSpeed * driveConstants.metresPerRotation);
                turnSets[i] = turnPIDController.calculate(positions[i], targetStates[i].angle.getRadians());
            }
        }

        //Update odometry
        Pose2d currPose = odometer.update(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
            new SwerveModulePosition(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModulePosition(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModulePosition(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModulePosition(velocities[3], Rotation2d.fromRadians(positions[3]))});
        
        //In theory this shouldn't ever happen (where output is false but input was true), but it never hurts to be safe
        while(outputUpdateEntry.get()) {
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
        //Update SmartDashboard with odometer values
        SmartDashboard.putNumber("Odometer X", currPose.getX());
        SmartDashboard.putNumber("Odometer Y", currPose.getY());
        SmartDashboard.putNumber("Odometer Reduced Angle", Math.IEEEremainder(currPose.getRotation().getDegrees(), 360));

        //Update NetworkTable with new values
        driveSetPublisher.set(driveSets);
        turnSetPublisher.set(turnSets);
        outputUpdateEntry.set(true);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometer(double[] velocities, double[] positions, double reducedAngle, Pose2d currPose) {
        odometer.resetPosition(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
            new SwerveModulePosition(velocities[0], Rotation2d.fromRadians(positions[0])), 
            new SwerveModulePosition(velocities[1], Rotation2d.fromRadians(positions[1])), 
            new SwerveModulePosition(velocities[2], Rotation2d.fromRadians(positions[2])), 
            new SwerveModulePosition(velocities[3], Rotation2d.fromRadians(positions[3]))}, currPose);
    }
}
