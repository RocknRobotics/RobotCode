package frc.robot.RoboRio;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RoboRio.Swerve.SwerveMaster;

public class Robot extends TimedRobot {
  //The PS4Controller for driving
  private PS4Controller driveController;
  //The current factor to multiply driveController inputs by -> 0, 0.25, 0.5, 0.75, or 1
  private double driveControllerFactor;
  //Self-explanatory
  private SwerveMaster mySwerveMaster;

  //A NetworkTable instance---for ease of use
  private NetworkTableInstance myInstance;
  //Publishes what method the Jetson/Laptop should be running (generally should match the current mode---robotInit, autoPeriodic, etc.)
  private StringPublisher currentMethodPublisher;
  
  @Override
  public void robotInit() {
    //Gets global default instance
      myInstance = NetworkTableInstance.getDefault();
      //Tells it to act as a NetworkTables 4 client with the given string identifying it
      myInstance.startClient4("rio");
      //Sets the server name to the laptopIPAddress (since that's the hostname), 
      //and port4 tells it to use the default port for NetworkTables 4
      myInstance.setServer(Constants.laptopIPAddress, NetworkTableInstance.kDefaultPort4);
      //Tells SmartDashboard to use myInstance as it's instance
      SmartDashboard.setInstance(myInstance);

    currentMethodPublisher = myInstance.getStringTopic("/rio/currentMethod").publish();
    currentMethodPublisher.set("robotInit");

    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 1d;
    mySwerveMaster = new SwerveMaster(myInstance);
  }

  //Every 20ms
  @Override
  public void robotPeriodic() {
    currentMethodPublisher.set("robotPeriodic");
  }

  @Override
  public void autonomousInit() {
    currentMethodPublisher.set("autonomousInit");
  }

  @Override
  public void autonomousPeriodic() {
    currentMethodPublisher.set("autonomousPeriodic");
  }

  @Override
  public void teleopInit() {
    currentMethodPublisher.set("teleopInit");

    if(driveController.getTouchpadPressed()) {
      driveControllerFactor = 0d;
    } else if(driveController.getSquareButtonPressed()) {
      driveControllerFactor = 0.25d;
    } else if(driveController.getCrossButtonPressed()) {
      driveControllerFactor = 0.5d;
    } else if(driveController.getCircleButtonPressed()) {
      driveControllerFactor = 0.75d;
    } else if(driveController.getTriangleButtonPressed()) {
      driveControllerFactor = 1d;
    }

    mySwerveMaster.update(driveController, driveControllerFactor);
  }

  @Override
  public void teleopPeriodic() {
    currentMethodPublisher.set("teleopPeriodic");
  }

  @Override
  public void disabledInit() {
    currentMethodPublisher.set("disabledInit");
  }

  @Override
  public void disabledPeriodic() {
    currentMethodPublisher.set("disabledPeriodic");
  }

  @Override
  public void testInit() {
    currentMethodPublisher.set("testInit");
  }

  @Override
  public void testPeriodic() {
    currentMethodPublisher.set("testPeriodic");
  }
}