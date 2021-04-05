package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowercellDetector extends SubsystemBase {
    private NetworkTable detectorTable = NetworkTableInstance.getDefault().getTable("powercellDetector");
    // private NetworkTable detectorTable = NetworkTableInstance.create().getTable("powercellDetector");
    // private NetworkTable detectorTable;
    private NetworkTableEntry tn = detectorTable.getEntry("tn");
    private NetworkTableEntry tr = detectorTable.getEntry("tr");
    private NetworkTableEntry tx = detectorTable.getEntry("tx");
    private NetworkTableEntry ty = detectorTable.getEntry("ty");
    private NetworkTableEntry ttheta = detectorTable.getEntry("ttheta");

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
        System.out.println(tn.exists());
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
     * Gets the angle the closest powercell makes with the camera. Negative is to
     * the left and positive is to the right
     * 
     * NOTE: if it doesn't see any powercells, this will return 0
     * 
     * @return angle of powercell - DEGREES
     */
    public double getHorizontalAngleDegrees() {
        return ttheta.getDouble(0);
    }

    /**
     * Gets the angle the closest powercell makes with the camera. Negative is to
     * the left and positive is to the right
     * 
     * NOTE: if it doesn't see any powercells, this will return 0
     * 
     * @return angle of powercell - RADIANS
     */
    public double getHorizontalAngleRadians() {
        return Units.degreesToRadians(getHorizontalAngleDegrees());
    }

    /**
     * Gets the X value of the position of the powercell assuming the camera is at
     * (0, 0) and facing positive X, meaning positive Y is to it's right
     * 
     * NOTE: if it doesn't see any powercells, this will return 0
     * 
     * @return X value - METERS
     */
    public double getPowercellX() {
        return tx.getDouble(0);
    }

    /**
     * Gets the Y value of the position of the powercell assuming the camera is at
     * (0, 0) and facing positive X, meaning positive Y is to it's right
     * 
     * NOTE: if it doesn't see any powercells, this will return 0
     * 
     * @return Y value - METERS
     */
    public double getPowercellY() {
        return ty.getDouble(0);
    }
}