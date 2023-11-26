package frc.robot.RoboRio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    //The width of the robot left to right in metres
    public static final double robotWidth = 0d; //TODO
    //The height of the robot top to bottom in metres
    public static final double robotHeight = 0d; //TODO
    //Length of the diagonal of the robot
    public static final double robotDiagonal = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotHeight, 2));

    public static final class talonConstants {
        //Update rate of the talons' position/velocity/acceleration in Hz 
        //(range 4-1000, although setting it to 0 turns it off (please don't do that))
        public static final int talonUpdateFrequency = 50;

        public static final class turnConstants {
            //PID controller position constant
            public static final double kP = 0.5; //Change this?
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
            public static final double stopBelowThisVelocity = 0.001d; //Make it higher? Or is it good?
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

    public static final class TrajectoryConstants {
        //The max (translational) acceleration to use for trajectory generation in metres/(second^2)
        public static final double maxTranslationalAcceleration = 1d; //TODO
        //The max angular acceleration to use in the drive controller constraints (see below) (radians/(second^2))
        public static final double maxAngularAcceleration = 1d; //TODO

        //IMPORTANT: Treat all coordinates with respect to the blue alliance and the bottom right corner (since frc is stupid and makes it so
        //that x is up/down and y is left/right except left is positive cause why not)
        //The minimum x values of each of the out of bounds areas, metres
        public static final double outXMin[] = new double[]{}; //TODO
        //The minimum y values of each of the out of bounds areas, metres
        public static final double outYMin[] = new double[]{}; //TODO
        //The maximum x values of each of the out of bounds areas, metres
        public static final double outXMax[] = new double[]{}; //TODO
        //The maximum y values of each of the out of bounds areas, metres
        public static final double outYMax[] = new double[]{}; //TODO

        //The length of the field (up/down) in metres
        //X direction
        public static final double fieldLength = 0d; //TODO
        //The width of the field (left/right) in metres
        //Y direction
        public static final double fieldWidth = 0d; //TODO

        //Since the trajectories are calculated using the center of the robot, then the diagonal / 2 will be the most avoidance we theoretically need
        public static final double robotAvoidance = robotDiagonal / 2d;

        //The number of points to initialize in the masterpoints array in the x direction
        public static final int xSteps = 100; //Mess around with this? Not sure how high we can/should go
        //The number of points to initialize in the masterpoints array in the y direction
        public static final int ySteps = 100;
        //Whether or not to use bounds when calculating nodes
        public static final boolean useNodeBounds = true; //Set to false?

        //The tolerance for determining whether or not the controller is at a given reference point (metres, metres, radians)
        public static final Pose2d controllerTolerance = new Pose2d(0d, 0d, new Rotation2d(0d)); //TODO
        //PID stuff for the holonomic drive controller
        //The kp, ki, and kd to use for the x part of the controller
        public static final double kX[] = new double[]{0d, 0d, 0d}; //TODO
        //Above but for the y part
        public static final double kY[] = new double[]{0d, 0d, 0d}; //TODO
        //Above for angle plus a trapezoidal constraints since it needs that too for the angle apparently
        public static final double kAngle[] = new double[]{0d, 0d, 0d}; //TODO
        //The trapezoidal angle constraints
        public static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAcceleration);
    }
}
