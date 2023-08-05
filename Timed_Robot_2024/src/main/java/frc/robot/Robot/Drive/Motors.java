package frc.robot.Robot.Drive;

import java.io.FileReader;
import java.io.IOException;
import java.util.EnumSet;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Constants;

public class Motors {
    //Change the type of motor (CANSparkMax) if necessary
    public CANSparkMax leftUp;
    public CANSparkMax rightUp;
    public CANSparkMax rightDown;
    public CANSparkMax leftDown;

    public Motors(CANSparkMax leftUp, boolean invertLeftUp, CANSparkMax rightUp, boolean invertRightUp, 
    CANSparkMax rightDown, boolean invertRightDown, CANSparkMax leftDown, boolean invertLeftDown) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        this.leftUp = leftUp;
        this.rightUp = rightUp;
        this.rightDown = rightDown;
        this.leftDown = leftDown;

        this.leftUp.setInverted(invertLeftUp);
        this.rightUp.setInverted(invertRightUp);
        this.rightDown.follow(rightUp, invertRightDown);
        this.leftDown.follow(leftUp, invertLeftDown);

        this.leftUp.setClosedLoopRampRate(Constants.closedLoopRampRate);
        this.rightUp.setClosedLoopRampRate(Constants.closedLoopRampRate);
        this.rightDown.setClosedLoopRampRate(Constants.closedLoopRampRate);
        this.leftDown.setClosedLoopRampRate(Constants.closedLoopRampRate);

        //Gets a string relating to the configuration using the current values of the motors
        String currentConfig = "";
        //Gets the string relating to the past configuration of the motors
        String fileConfig = "";
        
        try {
            FileReader configReader = new FileReader("src\\main\\configs\\MotorConfig.txt");

            for(int i = configReader.read(); i != -1; i = configReader.read()) {
                fileConfig += i;
            }

            configReader.close();

        } catch(IOException e) {
            e.printStackTrace();
        }

        //Add any relevant information to the config
        currentConfig = "" + leftUp.getInverted() + rightUp.getInverted() + rightDown.getInverted() + leftDown.getInverted() + 
        leftUp.isFollower() + rightUp.isFollower() + rightDown.isFollower() + leftDown.isFollower() +
        leftUp.getClosedLoopRampRate() + rightUp.getClosedLoopRampRate() + rightDown.getClosedLoopRampRate() + leftDown.getClosedLoopRampRate();

        //All this is to reduce the number of times the flash is burned by checking if the settings have changed
        if(!currentConfig.equals(fileConfig)) {
            leftUp.burnFlash();
            rightUp.burnFlash();
            rightDown.burnFlash();
            leftDown.burnFlash();

            BooleanSubscriber waitChecker = inst.getBooleanTopic("/laptop/fileWrite/waiting").subscribe(false);
            if(!waitChecker.get()) {
                inst.getStringTopic("/laptop/fileWrite/content").publish().set(currentConfig);
                inst.getStringTopic("/laptop/fileWrite/path").publish().set("src\\main\\configs\\MotorConfig.txt");
            } else {
                //Compiler was yelling at me that currentConfig needed to be final/effectively final when it was inside waitListener
                final String tempCurrentConfig = currentConfig;

                int waitListener = inst.addListener(inst.getTopic("/laptop/fileWrite/waiting"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
                    if(event.is(NetworkTableEvent.Kind.kValueAll) && waitChecker.get()) {
                        inst.getStringTopic("/laptop/fileWrite/content").publish().set(tempCurrentConfig);
                        inst.getStringTopic("/laptop/fileWrite/path").publish().set("src\\main\\configs\\MotorConfig.txt");

                        waitChecker.close();
                        this.close();
                    }
                });

                waitListener = waitListener + 0;
            }
        }
    }

    public void update(double speed, double turn) {
        leftUp.set(speed + turn);
        rightUp.set(speed - turn);
    }

    public void close() {
        leftUp.close();
        rightUp.close();
        rightDown.close();
        leftDown.close();
    }
}
