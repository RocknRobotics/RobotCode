package frc.robot.RoboRio.Swerve;

import frc.robot.RoboRio.Constants;
import frc.robot.RoboRio.Constants.talonConstants.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.I2C.Port;

public class SwerveMaster {
    private SwerveModule leftUpModule;
    private SwerveModule leftDownModule;
    private SwerveModule rightUpModule;
    private SwerveModule rightDownModule;

    private AHRS accelerometer;

    private SwerveDriveOdometry odometer;

    public SwerveMaster() {
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

        odometer = new SwerveDriveOdometry(driveConstants.driveTalonKinematics, new Rotation2d(0d), this.getModulePositions(), 
        new Pose2d(0d, 0d, new Rotation2d(0d)));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            new SwerveModulePosition(leftUpModule.getState().speedMetersPerSecond, leftUpModule.getState().angle), 
            new SwerveModulePosition(leftDownModule.getState().speedMetersPerSecond, leftDownModule.getState().angle), 
            new SwerveModulePosition(rightUpModule.getState().speedMetersPerSecond, rightUpModule.getState().angle), 
            new SwerveModulePosition(rightDownModule.getState().speedMetersPerSecond, rightDownModule.getState().angle)};
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

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometer(Pose2d currPose) {
        odometer.resetPosition(this.getRotation2d(), getModulePositions(), currPose);
    }

    public void stop() {
        leftUpModule.stop();
        leftDownModule.stop();
        rightUpModule.stop();
        rightDownModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] targetStates) {
        //Scales the targetStates in case it's not physically possible (for example it's impossible to go the true full move velocity 
        //and true full rotational velocity at the same time)
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, driveConstants.driveTalonKinematics.toChassisSpeeds(
        new SwerveModuleState[]{leftUpModule.getState(), leftDownModule.getState(), rightUpModule.getState(), rightDownModule.getState()}), 
        driveConstants.maxSpeed, Constants.maxTranslationalSpeed, Constants.maxAngularSpeed);
        //Set the states
        leftUpModule.setState(targetStates[0]);
        leftDownModule.setState(targetStates[1]);
        rightUpModule.setState(targetStates[2]);
        rightDownModule.setState(targetStates[3]);
    }

    public void update(PS4Controller input) {
        //Converts PS4 joystick inputs into field-relative speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(input.getLeftX() * Constants.maxTranslationalSpeed, 
        input.getLeftY() * Constants.maxTranslationalSpeed, input.getRightX() * Constants.maxTranslationalSpeed, this.getRotation2d());
        //Converts those speeds to targetStates since I'm not a monster who puts everything on one line (I had to resist the urge to)
        SwerveModuleState[] targetStates = driveConstants.driveTalonKinematics.toSwerveModuleStates(speeds);

        setModuleStates(targetStates);
    }
}
