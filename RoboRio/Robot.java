package frc.robot.RoboRio;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.RoboRio.Swerve.SwerveMaster;

public class Robot extends TimedRobot {
  private PS4Controller driveController;
  private SwerveMaster mySwerveMaster;
  
  @Override
  public void robotInit() {
    driveController = new PS4Controller(Constants.driveControllerPort);
    mySwerveMaster = new SwerveMaster();
  }

  //Every 20ms
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    mySwerveMaster.update(driveController);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}