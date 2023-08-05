// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Robot;

import java.util.EnumSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Drive.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  MotorUpdater myMotorUpdater;
  PS4Controller myDriveController;
  CANSparkMax leftUpCanSparkMax;
  CANSparkMax rightUpCanSparkMax;
  CANSparkMax rightDownCanSparkMax;
  CANSparkMax leftDownCanSparkMax;
  String motorControllerFile;
  BooleanSubscriber raspberryWaiting;
  StringPublisher raspberryInstructions;
  int raspberryInstructionsListener;
  BooleanSubscriber laptopWaiting;
  StringPublisher laptopInstructions;
  int laptopInstructionsListener;
  StringPublisher robotIsOn;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startClient4("team3692-frc2024");
    inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
    SmartDashboard.setNetworkTableInstance(inst);

    raspberryWaiting = inst.getBooleanTopic("/raspberry/waiting").subscribe(false);
    raspberryInstructions = inst.getStringTopic("/raspberry/instructions").publish();
    
    laptopWaiting = inst.getBooleanTopic("/laptop/waiting").subscribe(false);
    laptopInstructions = inst.getStringTopic("/laptop/instructions").publish();
    
    robotIsOn = inst.getStringTopic("/robot/isOn").publish();

    myDriveController = new PS4Controller(Constants.driveControllerPort);
    leftUpCanSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    rightUpCanSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    rightDownCanSparkMax = new CANSparkMax(3, MotorType.kBrushless);
    leftDownCanSparkMax = new CANSparkMax(4, MotorType.kBrushless);
    motorControllerFile = "src\\main\\configs\\NoahDriveConfig.txt";

    myMotorUpdater = new MotorUpdater(new MotorController(myDriveController, motorControllerFile), 
    new Motors(leftUpCanSparkMax, false, rightUpCanSparkMax, true, rightDownCanSparkMax, true, leftDownCanSparkMax, false), 
    false, motorControllerFile);

    if(!raspberryWaiting.get()) {
      raspberryInstructions.set("robotInit");
      inst.flush();
    } else {
      BooleanSubscriber waiting = inst.getBooleanTopic("/raspberry/waiting").subscribe(false);

      int waitListener = inst.addListener(inst.getTopic("/raspberry/waiting"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        if(event.is(NetworkTableEvent.Kind.kValueAll) && waiting.get()) {
          raspberryInstructions.set("robotInit");
          inst.flush();

          waiting.close();
          this.close();
        }
      });

      waitListener = waitListener + 0;
    }

    if(!laptopWaiting.get()) {
      laptopInstructions.set("robotInit");
      inst.flush();
    } else {
      BooleanSubscriber waiting = inst.getBooleanTopic("/laptop/waiting").subscribe(false);

      int waitListener = inst.addListener(inst.getTopic("/laptop/waiting"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        if(event.is(NetworkTableEvent.Kind.kValueAll) && waiting.get()) {
          laptopInstructions.set("robotInit");
          inst.flush();

          waiting.close();
          this.close();
        }
      });
      
      waitListener = waitListener + 0;
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    myMotorUpdater.close();
    SmartDashboard.putString("Raspberry Instructions", "Close");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void instruct(String instructions) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    if(!raspberryWaiting.get()) {
      raspberryInstructions.set(instructions);
      inst.flush();
    } else {
      BooleanSubscriber waiting = inst.getBooleanTopic("/raspberry/waiting").subscribe(false);

      int waitListener = inst.addListener(inst.getTopic("/raspberry/waiting"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        if(event.is(NetworkTableEvent.Kind.kValueAll) && waiting.get()) {
          raspberryInstructions.set(instructions);
          inst.flush();

          waiting.close();
          this.close();
        }
      });

      waitListener = waitListener + 0;
    }

    if(!laptopWaiting.get()) {
      laptopInstructions.set(instructions);
      inst.flush();
    } else {
      BooleanSubscriber waiting = inst.getBooleanTopic("/laptop/waiting").subscribe(false);

      int waitListener = inst.addListener(inst.getTopic("/laptop/waiting"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        if(event.is(NetworkTableEvent.Kind.kValueAll) && waiting.get()) {
          laptopInstructions.set(instructions);
          inst.flush();

          waiting.close();
          this.close();
        }
      });
      
      waitListener = waitListener + 0;
    }
  }
}
