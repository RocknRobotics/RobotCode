package frc.robot.Raspberry;

import java.util.EnumSet;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryMain {
    static BooleanPublisher waitConfirmer;
    static StringSubscriber instructionReader;
    static int instructionListener;
    static RaspberryMaster master;
    static int closeListener;
    public static void main(String[] args) {
        master = new RaspberryMaster();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        waitConfirmer = inst.getBooleanTopic("/raspberry/waiting").publish();
        instructionReader = inst.getStringTopic("/raspberry/instructions").subscribe("");
        instructionListener = inst.addListener(inst.getTopic("/raspberry/instructions"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                String instruction = instructionReader.get();

                if(instruction.equals("robotInit")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.robotInit();
                } else if(instruction.equals("robotPeriodic")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.robotPeriodic();
                } else if(instruction.equals("autonomousInit")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.autonomousInit();
                } else if(instruction.equals("autonomousPeriodic")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.autonomousPeriodic();
                } else if(instruction.equals("teleopInit")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.teleopInit();
                } else if(instruction.equals("teleopPeriodic")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.teleopPeriodic();
                } else if(instruction.equals("disabledInit")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.disabledInit();
                } else if(instruction.equals("disabledPeriodic")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.disabledPeriodic();
                } else if(instruction.equals("testInit")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.testInit();
                } else if(instruction.equals("testPeriodic")) {
                    waitConfirmer.set(false);
                    inst.flush();
                    master.testPeriodic();
                }

                waitConfirmer.set(true);
                inst.flush();
            }
        });

        closeListener = inst.addListener(inst.getTopic("/robot/isOn"), EnumSet.of(NetworkTableEvent.Kind.kDisconnected), event -> {
            if(event.is(NetworkTableEvent.Kind.kDisconnected)) {
                master.close();
            }
        });
    }
}
