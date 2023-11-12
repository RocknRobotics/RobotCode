package frc.robot.Laptop;

import java.util.ArrayList;
import java.util.EnumSet;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StringSubscriber;

public class LaptopMaster {
    @SuppressWarnings("unused")
    private FileUpdater myFileUpdater;
    private SwerveCalculator mySwerveCalculator;

    private NetworkTableInstance myInstance;
    //Listens for when a new method instruction is sent
    @SuppressWarnings("unused")
    private NetworkTableListener currentMethodListener;
    private StringSubscriber currentMethodSubscriber;

    //Will be used to keep sort of the methods to run
    //Acts as a psuedo priority queue
    private ArrayList<String> methodNames;

    //Runs the threads by taking the first element of the above list
    private Thread methodRunner;

    public LaptopMaster() {
        //Gets global default instance
        myInstance = NetworkTableInstance.getDefault();
        //Tells it to act as a NetworkTables 4 client with the given string identifying it
        myInstance.startClient4("laptop");
        //Sets the team, and port4 tells it to use the default port for NetworkTables 4
        myInstance.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);

        myFileUpdater = new FileUpdater(myInstance);
        mySwerveCalculator = new SwerveCalculator(myInstance);

        methodNames = new ArrayList<String>();

        currentMethodSubscriber = myInstance.getStringTopic("/rio/currentMethod").subscribe("default");
        currentMethodListener = NetworkTableListener.createListener(myInstance.getStringTopic("/rio/currentMethod"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                //In case the method changes for whatever reason
                String temp = currentMethodSubscriber.get();
                //Checks if the method is already in the list
                boolean notIn = true;

                for(String method: methodNames) {
                    if(method.equals(temp)) {
                        notIn = false;
                    }
                }

                if(notIn) {
                    //If the method is an initialization method
                    if(temp.endsWith("Init")) {
                        //Remove all periodic methods
                        for(int i = methodNames.size() - 1; i >= 0; i--) {
                            if(methodNames.get(i).endsWith("Periodic")) {
                                methodNames.remove(i);
                            } else {
                                break;
                            }
                        }
                    }
                    //Either way add the new function at the end, regardless of if it's an initialization or periodic function
                    methodNames.add(temp);
                }
            }
        });

        //Thread to execute the methods in the order they're in the methodNames arrayList
        methodRunner = new Thread(() -> {
            while(true) {
                while(methodNames.size() == 0) {
                    try {
                        Thread.sleep(10);
                    } catch(InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                try {
                    Method method = this.getClass().getDeclaredMethod(methodNames.remove(0), Void.class);

                    try {
                        method.invoke(this);
                    } catch(IllegalAccessException e) {
                        e.printStackTrace();
                    } catch(InvocationTargetException d) {
                        d.printStackTrace();
                    }
                } catch(NoSuchMethodException e) {
                    e.printStackTrace();
                }
            }
        });
        methodRunner.start(); //TODO change priority? + change other configs?
    }

    public void robotInit() {}

    public void robotPeriodic() {
        mySwerveCalculator.robotPeriodic();
    }

    public void autonomousInit() {}

    public void autonomousPeriodic() {}

    public void teleopInit() {}

    public void teleopPeriodic() {
        mySwerveCalculator.testPeriodic();
    }

    public void disabledInit() {
        mySwerveCalculator.disabledInit();
    }

    public void disabledPeriodic() {}

    public void testInit() {}

    public void testPeriodic() {}
}
