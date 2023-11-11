package frc.robot.RoboRio.Swerve;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RoboRio.Constants.talonConstants;
import frc.robot.RoboRio.Constants.talonConstants.*;

//The class is used to represent a drive talon and turn talon that are part of one the swerve modules
public class SwerveModule {
    //The TalonFX motors---they have built-in encoders, so no need to create encoder objects
    private TalonFX driveTalon;
    private TalonFX turnTalon;

    //Enables continious input I think? I'm writing this using another person's code as a guide---I'll mess around with changing
    //this once we have the swerve drive built
    private PIDController turnPIDController;

    public SwerveModule(int driveTalonID, int turnTalonID, boolean driveTalonInvert, boolean turnTalonInvert) {
        driveTalon = new TalonFX(driveTalonID);
        turnTalon = new TalonFX(turnTalonID);

        driveTalon.getPosition().setUpdateFrequency(talonConstants.talonUpdateFrequency, 1d);
        turnTalon.getPosition().setUpdateFrequency(talonConstants.talonUpdateFrequency, 1d);
        driveTalon.getVelocity().setUpdateFrequency(talonConstants.talonUpdateFrequency, 1d);
        turnTalon.getVelocity().setUpdateFrequency(talonConstants.talonUpdateFrequency, 1d);
        driveTalon.getAcceleration().setUpdateFrequency(talonConstants.talonUpdateFrequency, 1d);
        turnTalon.getAcceleration().setUpdateFrequency(talonConstants.talonUpdateFrequency, 1d);

        turnPIDController = new PIDController(turnConstants.kP, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    //Returns a SwerveModuleState representation of this SwerveModule
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(this.getTurnPosition()));
    }

    public void setState(SwerveModuleState targetState) {
        if(Math.abs(targetState.speedMetersPerSecond) < driveConstants.stopBelowThisVelocity) {
            this.stop();
        } else {
            targetState = SwerveModuleState.optimize(targetState, new Rotation2d(this.getTurnPosition()));
            driveTalon.set(targetState.speedMetersPerSecond / (driveConstants.maxSpeed * driveConstants.metresPerRotation));
            turnTalon.set(turnPIDController.calculate(this.getTurnPosition(), targetState.angle.getRadians()));
        }
    }

    public void stop() {
        driveTalon.set(0);
        turnTalon.set(0);
    }

    //Metres position of the drive talon
    public double getDrivePosition() {
        return driveTalon.getPosition().getValueAsDouble() * driveConstants.metresPerRotation;
    }

    //Radians position of the turn talon
    public double getTurnPosition() {
        return turnTalon.getPosition().getValueAsDouble() * turnConstants.radsPerRotation;
    }

    //Metres/second velocity of the drive talon
    public double getDriveVelocity() {
        return driveTalon.getVelocity().getValueAsDouble() * driveConstants.metresPerRotation;
    }

    //Radians/second velocity of the turn talon
    public double getTurnVelocity() {
        return turnTalon.getVelocity().getValueAsDouble() * turnConstants.radsPerRotation;
    }

    //Metres/(second^2) acceleration of the drive talon
    public double getDriveAcceleration() {
        return driveTalon.getAcceleration().getValueAsDouble() * driveConstants.metresPerRotation;
    }

    //Radians/(second^2) acceleration of the turn talon
    public double getTurnAcceleration() {
        return turnTalon.getAcceleration().getValueAsDouble() * turnConstants.radsPerRotation;
    }
}
