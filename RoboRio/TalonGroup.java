package frc.robot.RoboRio;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonGroup {
    private TalonFX leftUpTurn;
    private TalonFX leftUpDrive;
    private TalonFX leftDownTurn;
    private TalonFX leftDownDrive;
    private TalonFX rightUpTurn;
    private TalonFX rightUpDrive;
    private TalonFX rightDownTurn;
    private TalonFX rightDownDrive;

    private double fakeWheelRadius;
    private double fakeWheelCircumference;

    //We can treat the centres of the drive wheels as forming a square inscribed inside a circle (whose circumference is the above),
    //with the wheels' centres being at a certain angle on the circle
    private double leftUpCircleAngle;
    private double leftDownCircleAngle;
    private double rightUpCircleAngle;
    private double rightDownCircleAngle;

    private double timeSinceSet;

    private NetworkTableInstance inst;

    private double currX;
    private double currY;
    private double currAngle;

    public TalonGroup() {
        leftUpTurn = new TalonFX(Constants.leftUpTurnTalonID);
        leftUpDrive = new TalonFX(Constants.leftUpDriveTalonID);
        leftDownTurn = new TalonFX(Constants.leftDownTurnTalonID);
        leftDownDrive = new TalonFX(Constants.leftDownDriveTalonID);
        rightUpTurn = new TalonFX(Constants.rightUpTurnTalonID);
        rightUpDrive = new TalonFX(Constants.rightUpDriveTalonID);
        rightDownTurn = new TalonFX(Constants.rightDownTurnTalonID);
        rightDownDrive = new TalonFX(Constants.rightDownDriveTalonID);

        leftUpTurn.setInverted(Constants.leftUpTurnTalonInverted);
        leftUpDrive.setInverted(Constants.leftUpDriveTalonInverted);
        leftDownTurn.setInverted(Constants.leftDownTurnTalonInverted);
        leftDownDrive.setInverted(Constants.leftDownDriveTalonInverted);
        rightUpTurn.setInverted(Constants.rightUpTurnTalonInverted);
        rightUpDrive.setInverted(Constants.rightUpDriveTalonInverted);
        rightDownTurn.setInverted(Constants.rightDownTurnTalonInverted);
        rightDownDrive.setInverted(Constants.rightDownDriveTalonInverted);

        fakeWheelRadius = Constants.driveWheelCentreDiagonal / 2d;
        fakeWheelCircumference = Constants.driveWheelCentreDiagonal * Math.PI;

        leftUpCircleAngle = 0.75 * Math.PI;
        leftDownCircleAngle = 1.25 * Math.PI;
        rightUpCircleAngle = 0.25 * Math.PI;
        rightDownCircleAngle = 1.75 * Math.PI;

        inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        currX = 0d;
        currY = 0d;
        currAngle = 0d;
    }

    public void set(double xAxis, double yAxis, double turnAxis) {
    }
}