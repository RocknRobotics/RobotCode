package frc.robot.RoboRio;

public class Constants {
    public static final int leftUpTurnTalonID = 0;
    public static final int leftUpDriveTalonID = 1;
    public static final int leftDownTurnTalonID = 2;
    public static final int leftDownDriveTalonID = 3;
    public static final int rightUpTurnTalonID = 4;
    public static final int rightUpDriveTalonID = 5;
    public static final int rightDownTurnTalonID = 6;
    public static final int rightDownDriveTalonID = 7;

    public static final boolean leftUpTurnTalonInverted = false;
    public static final boolean leftUpDriveTalonInverted = false;
    public static final boolean leftDownTurnTalonInverted = false;
    public static final boolean leftDownDriveTalonInverted = false;
    public static final boolean rightUpTurnTalonInverted = false;
    public static final boolean rightUpDriveTalonInverted = false;
    public static final boolean rightDownTurnTalonInverted = false;
    public static final boolean rightDownDriveTalonInverted = false;
    
    //In metres
    public static final double driveWheelCircumference = 0.0;
    //The number of drive talon rotations that occur in order for one drive wheel rotation to happen
    public static final double driveTalonRotationRatio = 0.0;
    //Don't need circumference since 1 rev = 360 degrees
    public static final double turnTalonRotationRatio = 0.0;

    //In metres, the distance between the centres of two diagonal wheels
    public static final double driveWheelCentreDiagonal = 0.0;

}