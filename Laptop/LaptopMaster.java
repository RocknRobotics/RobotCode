package frc.robot.Laptop;

public class LaptopMaster {
    @SuppressWarnings("unused")
    private FileUpdater myFileUpdater;
    @SuppressWarnings("unused")
    private SwerveCalculator mySwerveCalculator;

    public LaptopMaster() {
        myFileUpdater = new FileUpdater();
        mySwerveCalculator = new SwerveCalculator();
    }
}
