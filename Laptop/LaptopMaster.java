package frc.robot.Laptop;

import java.util.ArrayList;
import java.util.EnumSet;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LaptopMaster {
    private FileUpdater myFileUpdater;
    private SwerveCalculator mySwerveCalculator;
    private TrajectoryHelper myTrajectoryHelper;

    private NetworkTableInstance myInstance;
    //Listens for when the rio connects/disconnects
    @SuppressWarnings("unused")
    private NetworkTableListener connectionChangeListener;
    //Listens for when a new method instruction is sent
    @SuppressWarnings("unused")
    private NetworkTableListener currentMethodListener;
    private StringSubscriber currentMethodSubscriber;

    //Will be used to keep sort of the methods to run
    //Acts as a psuedo priority queue
    private ArrayList<String> methodNames;
    //The index of the where to insert the next init method (AKA the index after the last init method)
    private int lastInitMethodIndex;

    //This way it doesn't try and execute methods after closing
    private boolean executeMethods;

    public LaptopMaster() {
        myInstance = NetworkTableInstance.getDefault();
        myInstance.startServer("networktables.json", "", NetworkTableInstance.kDefaultPort3, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(myInstance);

        //If you look in the classes you should see this doesn't do much aside from assign the instance for them to use
        myFileUpdater = new FileUpdater(myInstance);
        mySwerveCalculator = new SwerveCalculator(myInstance);
        myTrajectoryHelper = new TrajectoryHelper(myInstance);

        methodNames = new ArrayList<String>();
        lastInitMethodIndex = 0;

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
        myFileUpdater.create();
        mySwerveCalculator.create();
        myTrajectoryHelper.create();

        currentMethodSubscriber = myInstance.getStringTopic("/rio/currentMethod").subscribe("default");
        currentMethodListener = NetworkTableListener.createListener(myInstance.getStringTopic("/rio/currentMethod"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                //In case the method changes for whatever reason
                String temp = currentMethodSubscriber.get();
                boolean isInit = temp.endsWith("Init");

                if(isInit) {
                    //Checks if the initialization function is already queued up
                    //If it is, change it to the end of the init functions
                    for(int i = 0; i < lastInitMethodIndex; i++) {
                        if(methodNames.get(i).equals(temp)) {
                            methodNames.remove(i);
                            lastInitMethodIndex--;
                            break;
                        }
                    }

                    //Add the init function
                    methodNames.add(lastInitMethodIndex, temp);
                    lastInitMethodIndex++;
                        
                    //Remove the periodic functions after it
                    for(int i = methodNames.size() - 1; i >= lastInitMethodIndex; i--) {
                        methodNames.remove(i);
                    }
                } else {
                    //Checks if the method is already in the list
                    boolean notIn = true;

                    for(int i = lastInitMethodIndex; i < methodNames.size(); i++) {
                        if(methodNames.get(i).equals(temp)) {
                            notIn = false;
                            break;
                        }
                    }

                    if(notIn) {
                        methodNames.add(temp);
                    }
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
                String temp = methodNames.remove(0);

                if(temp.endsWith("Init")) {
                    lastInitMethodIndex--;
                }

                //Take the first element in the queue and get the method it points to
                Method method = this.getClass().getDeclaredMethod(temp, Void.class);

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
        methodNames.clear();
        lastInitMethodIndex = 0;
        myFileUpdater.close();
        mySwerveCalculator.close();
        myTrajectoryHelper.close();
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
