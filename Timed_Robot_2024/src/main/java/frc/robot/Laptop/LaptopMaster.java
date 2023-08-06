package frc.robot.Laptop;

public class LaptopMaster {
    InformationComparer myInformationComparer;

    public LaptopMaster() {}

    public void robotInit() {
        myInformationComparer = new InformationComparer();
    }

    public void robotPeriodic() {

    }

    public void autonomousInit() {

    }

    public void autonomousPeriodic() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {

    }

    public void disabledInit() {

    }

    public void disabledPeriodic() {

    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void close() {
        myInformationComparer.close();
    }
}
