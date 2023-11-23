package frc.robot.Laptop;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RoboRio.Constants;
import frc.robot.RoboRio.Constants.talonConstants.driveConstants;
import frc.robot.RoboRio.Constants.talonConstants.turnConstants;

public class SwerveCalculator {
    //The inputs from the drive controller
    private DoubleArraySubscriber controllerInputsSubscriber;
    //The current velocites of each of the swerve modules
    private DoubleArraySubscriber driveVelocitiesSubscriber;
    //The current positions of the swerve modules
    private DoubleArraySubscriber turnPositionsSubscriber;
    //The reduced angle reported by the accelerometer
    private DoubleSubscriber reducedAngleSubscriber;
    //Publishes the values to set the drive Talons to
    private DoubleArrayPublisher driveSetPublisher;
    //Publishes the values to set the turn Talons to
    private DoubleArrayPublisher turnSetPublisher;
    //Reads whether or not there are currently update outputs waiting to be read (probably never needs to wait before sending)
    private BooleanEntry outputUpdateEntry;
    //Reads whether or not to reset the odometer with the current Pose2d given below
    private BooleanEntry resetOdometerEntry;
    //The Pose2d to reset the odometer using
    private DoubleArraySubscriber resetOdometerCurrentPoseSubscriber;

    private SwerveDriveOdometry odometer;

    //Enables continious input I think? I'm writing this using another person's code as a guide---I'll mess around with changing
    //this once we have the swerve drive built
    private PIDController turnPIDController;

    private NetworkTableInstance myInstance;

    public SwerveCalculator(NetworkTableInstance inst) {
        myInstance = inst;
    }

    public void create() {
        //Being a subscriber pretty much means it'll be rio side, since it's reading values from the rio
        controllerInputsSubscriber = myInstance.getDoubleArrayTopic("/rio/swerve/controller/inputs").subscribe(new double[]{0d, 0d, 0d});
        driveVelocitiesSubscriber = myInstance.getDoubleArrayTopic("/rio/swerve/drive/velocities").subscribe(new double[]{0d, 0d, 0d});
        turnPositionsSubscriber = myInstance.getDoubleArrayTopic("/rio/swerve/turn/positions").subscribe(new double[]{0d, 0d, 0d});
        reducedAngleSubscriber = myInstance.getDoubleTopic("/rio/swerve/accelerometer/reducedAngle").subscribe(0d);
        resetOdometerCurrentPoseSubscriber = myInstance.getDoubleArrayTopic("/rio/swerve/odometer/reset/currentPose").subscribe(new double[]{0d, 0d, 0d});
        //Laptop side publishers + entries
        driveSetPublisher = myInstance.getDoubleArrayTopic("/laptop/swerve/drive/set").publish();
        turnSetPublisher = myInstance.getDoubleArrayTopic("/laptop/swerve/turn/set").publish();
        outputUpdateEntry = myInstance.getBooleanTopic("/laptop/swerve/update/output").getEntry(false);
        resetOdometerEntry = myInstance.getBooleanTopic("/laptop/swerve/odometer/reset").getEntry(false);

        turnPIDController = new PIDController(turnConstants.kP, 0d, 0d);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void close() {
        controllerInputsSubscriber.close();
        controllerInputsSubscriber = null;
        driveVelocitiesSubscriber.close();
        driveVelocitiesSubscriber = null;
        turnPositionsSubscriber.close();
        turnPositionsSubscriber = null;
        reducedAngleSubscriber.close();
        reducedAngleSubscriber = null;
        resetOdometerCurrentPoseSubscriber.close();
        resetOdometerCurrentPoseSubscriber = null;
        driveSetPublisher.close();
        driveSetPublisher = null;
        turnSetPublisher.close();
        turnSetPublisher = null;
        outputUpdateEntry.close();
        outputUpdateEntry = null;
        resetOdometerEntry.close();
        resetOdometerEntry = null;
        turnPIDController.close();
        turnPIDController = null;

        odometer = null;
    }

    public void robotInit() {}

    public void robotPeriodic() {
        //Update SmartDashboard with odometer values
        Pose2d currPose = odometer.getPoseMeters();
        SmartDashboard.putNumber("Odometer X", currPose.getX());
        SmartDashboard.putNumber("Odometer Y", currPose.getY());
        SmartDashboard.putNumber("Odometer Reduced Angle", Math.IEEEremainder(currPose.getRotation().getDegrees(), 360));
    }

    public void autonomousInit() {}

    public void autonomousPeriodic() {}

    public void teleopInit() {}

    public void teleopPeriodic() {
        if(resetOdometerEntry.get()) {
            resetOdometer(driveVelocitiesSubscriber.get(), turnPositionsSubscriber.get(), reducedAngleSubscriber.get(), new Pose2d(
                resetOdometerCurrentPoseSubscriber.get()[0], resetOdometerCurrentPoseSubscriber.get()[1], new Rotation2d(resetOdometerCurrentPoseSubscriber.get()[2])));
            resetOdometerEntry.set(false);
        }

        teleopUpdate(controllerInputsSubscriber.get(), driveVelocitiesSubscriber.get(), turnPositionsSubscriber.get(), reducedAngleSubscriber.get());

        myInstance.flush();
    }

    public void disabledInit() {
        while(outputUpdateEntry.get()) {
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }

        driveSetPublisher.set(new double[]{0d, 0d, 0d, 0d});
        turnSetPublisher.set(new double[]{0d, 0d, 0d, 0d});
        outputUpdateEntry.set(true);

        myInstance.flush();
    }

    public void disabledPeriodic() {}

    public void testInit() {}

    public void testPeriodic() {}

    //Does the heavy lifting
    public void teleopUpdate(double[] inputs, double[] velocities, double[] positions, double reducedAngle) {
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
        odometer.update(Rotation2d.fromDegrees(reducedAngle), new SwerveModulePosition[]{
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
