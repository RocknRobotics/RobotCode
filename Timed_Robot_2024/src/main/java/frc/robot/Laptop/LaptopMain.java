package frc.robot.Laptop;

import java.io.FileWriter;
import java.io.IOException;
import java.util.EnumSet;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LaptopMain {
    static BooleanPublisher fileWaiting;
    static StringSubscriber fileContent;
    static StringSubscriber filePath;
    static int fileWriter;
    static BooleanPublisher waitConfirmer;
    static StringSubscriber instructionReader;
    static int instructionListener;
    static LaptopMaster master;
    static int closeListener;

    public static void main(String[] args) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        fileWaiting = inst.getBooleanTopic("/laptop/fileWrite/waiting").publish();
        filePath = inst.getStringTopic("/laptop/fileWrite/path").subscribe("");
        fileContent = inst.getStringTopic("/laptop/fileWrite/content").subscribe("");
        fileWriter = inst.addListener(inst.getTopic("/laptop/fileWrite"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                fileWaiting.set(false);
                inst.flush();
                String path = filePath.get();
                String content = fileContent.get();

                try {
                    FileWriter author = new FileWriter(path, false);
                    author.write(content, 0, content.length());
                    author.close();

                } catch(IOException e) {
                    e.printStackTrace();
                } finally {
                    fileWaiting.set(true);
                    inst.flush();
                }
            }
        });

        waitConfirmer = inst.getBooleanTopic("/laptop/waiting").publish();
        instructionReader = inst.getStringTopic("/laptop/instructions").subscribe("");
        instructionListener = inst.addListener(inst.getTopic("/laptop/instructions"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
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
