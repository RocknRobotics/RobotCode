package frc.robot.Jetson;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.RoboRio.Constants;

public class JetsonMaster {
    private NetworkTableInstance myInstance;
    //Listens for when the rio connects/disconnects
    private NetworkTableListener connectionChangeListener;
    //Listens for when a new method instruction is sent
    @SuppressWarnings("unused")
    private NetworkTableListener currentMethodListener;
    private StringSubscriber currentMethodSubscriber;

    //Will be used to keep sort of the methods to run
    //Acts as a psuedo priority queue
    private ArrayList<String> methodNames;

    private boolean executeMethods;

    public JetsonMaster() {
        //Gets global default instance
        myInstance = NetworkTableInstance.getDefault();
        //Tells it to act as a NetworkTables 4 client with the given string identifying it
        myInstance.startClient4("jetson");
        //Sets the server name to the laptopIPAddress (since that's the hostname), 
        //and port4 tells it to use the default port for NetworkTables 4
        myInstance.setServer(Constants.laptopIPAddress, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(myInstance);

        methodNames = new ArrayList<String>();

        //Need it to persist after rio dies, so we'll keep this listener in here instead of in the create method
        connectionChangeListener = NetworkTableListener.createConnectionListener(myInstance, true, event -> {
            if(event.is(NetworkTableEvent.Kind.kConnected)) {
                String name = event.connInfo.remote_id;

                if(name.equals("rio")) {
                    this.create();
                }
            } else if(event.is(NetworkTableEvent.Kind.kDisconnected)) {
                String name = event.connInfo.remote_id;

                if(name.equals("rio")) {
                    this.close();
                }
            }
        });
    }
    
    public void create() {
        executeMethods = true;

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

        //Start executing methods
        while(executeMethods) {
            //While I don't have a method
            while(methodNames.size() == 0) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            try {
                //Take the first element in the queue and get the method it points to
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
    }

    public void close() {
        executeMethods = false;
    }

    public void robotInit() {}

    public void robotPeriodic() {}

    public void autonomousInit() {}

    public void autonomousPeriodic() {}

    public void teleopInit() {}

    public void teleopPeriodic() {}

    public void disabledInit() {}

    public void disabledPeriodic() {}

    public void testInit() {}

    public void testPeriodic() {}
}
