package frc.robot.Laptop;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LaptopMain {

    public static void main(String[] args) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2024");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        
    }
}
