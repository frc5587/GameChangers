package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import Networ;/

public class PowercellDetector extends SubsystemBase {
    private NetworkTable detectorTable = NetworkTableInstance.getDefault().getTable("powercellDetector");
    private NetworkTableEntry tn = detectorTable.getEntry("tn");
    private NetworkTableEntry tr = detectorTable.getEntry("tr");
    private NetworkTableEntry tTheta = detectorTable.getEntry("tTheta");

    public PowercellDetector() {
        
    }

    /**
     * Gets the number of powercells the powercell detector sees
     * 
     * @return number of powercells
     */
    public int numPowercells() {
        return (int) tn.getDouble(0);
    }

    /**
     * Check if the powercell detector sees any powercells
     * 
     * @return if powercells are seen
     */
    public boolean seesPowercell() {
        return numPowercells() > 0;
    }

    /**
     * Gets the distance of the closest powercell. 
     * 
     * NOTE: if it doesn't see any powercells, this will return 0
     * 
     * @return distance to nearest powercell - METERS
     */
    public double getDistance() {
        return tr.getDouble(0);
    }

    /**
     * Gets the angle the closest powercell makes with the camera. Negative is to the left and positive is to the right
     * 
     * @return angle of powercell - DEGREES
     */
    public double getHorizontalAngleDegrees() {
        return tTheta.getDouble(0);
    }

    /**
     * Gets the angle the closest powercell makes with the camera. Negative is to the left and positive is to the right
     * 
     * @return angle of powercell - RADIANS
     */
    public double getHorizontalAngleRadians() {
        return Units.degreesToRadians(getHorizontalAngleDegrees());
    }
}