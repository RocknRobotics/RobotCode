package frc.robot.RoboRio;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/*
Stuff to do later:
--Mess around with changing kP---Maybe introduce kD and/or kI?
--Determine a good talonConstants/driveConstants/stopBelowThisVelocity value
--Change accelerometer/talon update frequency? May reach limit of usefulness depending on how often loops are executed
*/

public final class Constants {
    //The diamatre (in metres) of the wheels for driving
    public static final double driveWheelDiameter = 1d; //TODO
    
    //Update rate of the accelerometer, in Hz (range 4-200)
    public static final byte accelerometerUpdateFrequency = 50;

    //The physical max speed the robot can move in metres/second
    public static final double maxTranslationalSpeed = 1d; //TODO
    //The physical max speed the robot can rotate in radians/second
    public static final double maxAngularSpeed = 1d; //TODO

    //The port the drive controller is connected to
    public static final int driveControllerPort = 0; //TODO
    //The IP address of the laptop when it's connected to the network
    public static final String laptopIPAddress = ""; //TODO

    //The max acceleration to use for trajectory generation in metres/(second^2)
    public static final double maxAcceleration = 1d; //TODO

    //The width of the robot left to right in metres
    public static final double robotWidth = 0d; //TODO
    //The height of the robot top to bottom in metres
    public static final double robotHeight = 0d; //TODO

    //IMPORTANT: Treat all coordinates with respect to left corner of the blue alliance (that is, assume the origin to be that corner
    //The minimum x values of each of the out of bounds areas, metres
    public static final double outXMin[] = new double[]{};
    //The minimum y values of each of the out of bounds areas, metres
    public static final double outYMin[] = new double[]{};
    //The maximum x values of each of the out of bounds areas, metres
    public static final double outXMax[] = new double[]{};
    //The maximum y values of each of the out of bounds areas, metres
    public static final double outYMax[] = new double[]{};
    //The maximum x value of what constitutes the in bounds area, treating the minimum x value as 0 (so essentially the length/width)
    //Metres
    public static final double inXMax = 0d;
    //The maximum y value of what constitutes the in bounds area, treating the minimum y value as 0 (so essentially the length/width)
    //Metres
    public static final double inYMax = 0d;

    //The number of points to initialize in the masterpoints array in the x direction
    public static final int xSteps = 0;
    //The number of points to initialize in the masterpoints array in the y direction
    public static final int ySteps = 0;
    //Whether or not to use bounds when calculating nodes
    public static final boolean useNodeBounds = true;

    public static final class talonConstants {
        //Update rate of the talons' position/velocity/acceleration in Hz 
        //(range 4-1000, although setting it to 0 turns it off (please don't do that))
        public static final int talonUpdateFrequency = 50;

        public static final class turnConstants {
            //PID controller position constant
            public static final double kP = 0.5;
            //Gear ratio between the turn talons and the module?
            public static final double gearRatio = 1d / 1d; //TODO
            //The amount of radians per rotation of the turn talon (Size of "wheel" being rotated doesn't matter---One full rotation
            //of any size wheel equals 2 radians)
            public static final double radsPerRotation = gearRatio * 2 * Math.PI;
            //The physical max speed in rotations/second of the turn talons
            public static final double maxSpeed = 1d; //TODO

            //TODO
            public static final int leftUpID = 0;
            public static final int leftDownID = 1;
            public static final int rightUpID = 2;
            public static final int rightDownID = 3;

            //TODO
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = false;
        }

        public static final class driveConstants {
            //The gear ratio between the drive talons and the drive wheel---the amount of drive wheel rotations per talon rotation
            public static final double gearRatio = 1d / 1d; //TODO
            //The amount of metres traveled per rotation of the drive talon (Circumference of wheel * wheel rotation per talon rotation)
            public static final double metresPerRotation = gearRatio * driveWheelDiameter * Math.PI;
            //Units in metres/second, the velocity below which the swerve module motors will stop instead of going to a desired state
            public static final double stopBelowThisVelocity = 0.001d;
            //The physical max speed in rotations/second of the drive talons
            public static final double maxSpeed = 1d; //TODO

            //TODO
            public static final int leftUpID = 4;
            public static final int leftDownID = 5;
            public static final int rightUpID = 6;
            public static final int rightDownID = 7;

            //TODO
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = false;

            //The distance in metres between the left wheels and the right wheels
            public static final double leftToRightDistanceMetres = 1d; //TODO
            //Distance in metres between the "up" wheels and the "down" wheels
            public static final double upToDownDistanceMetres = 1d; //TODO

            //Order--- leftUp, leftDown, rightUp, rightDown
            public static final SwerveDriveKinematics driveTalonKinematics = new SwerveDriveKinematics(
                new Translation2d(upToDownDistanceMetres / 2d, -leftToRightDistanceMetres / 2d), 
                new Translation2d(-upToDownDistanceMetres / 2d, -leftToRightDistanceMetres / 2d), 
                new Translation2d(upToDownDistanceMetres / 2d, leftToRightDistanceMetres / 2d), 
                new Translation2d(-upToDownDistanceMetres / 2d, leftToRightDistanceMetres / 2d));
        }
    }
}
