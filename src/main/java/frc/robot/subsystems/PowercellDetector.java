package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowercellDetector extends SubsystemBase {
    private NetworkTable detectorTable = NetworkTableInstance.getDefault().getTable("powercellDetector");
    
    private NetworkTableEntry tn = detectorTable.getEntry("tn");
    private NetworkTableEntry tr = detectorTable.getEntry("tr");
    private NetworkTableEntry tx = detectorTable.getEntry("tx");
    private NetworkTableEntry ty = detectorTable.getEntry("ty");
    private NetworkTableEntry ttheta = detectorTable.getEntry("ttheta");

    private NetworkTableEntry trArr = detectorTable.getEntry("trArr");
    private NetworkTableEntry txArr = detectorTable.getEntry("txArr");
    private NetworkTableEntry tyArr = detectorTable.getEntry("tyArr");
    private NetworkTableEntry tthetaArr = detectorTable.getEntry("tthetaArr");

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

    /**
     * Gets all the data representing the closest powercell found to the robot
     * 
     * @return an instance containing the x, y, r, and theta, all relative to the camera
     */
    public Powercell getPowercell() {
        return new Powercell(getPowercellX(), getPowercellY(), getDistance(), getHorizontalAngleRadians());
    }

    /**
     * Gets a list of all currently seen powercells and returns them as a `List` of `Powercell`
     * 
     * @return all currently seen powercells
     */
    public List<Powercell> getAllPowercells() {
        List<Powercell> powercells = Arrays.asList();

        int nPowercells = numPowercells();
        double[] xArr = txArr.getDoubleArray(new double[0]);
        double[] yArr = tyArr.getDoubleArray(new double[0]);
        double[] rArr = trArr.getDoubleArray(new double[0]);
        double[] thetaArr = tthetaArr.getDoubleArray(new double[0]);

        assert nPowercells == xArr.length && xArr.length == yArr.length && yArr.length == rArr.length && rArr.length == thetaArr.length;

        for (int i = 0; i < nPowercells; i++) {
            powercells.add(new Powercell(xArr[i], yArr[i], rArr[i], thetaArr[i]));
        }

        return powercells;
    }

    public static class Powercell {
        public final double kX;
        public final double kY;
        public final double kR;
        public final double kTheta;

        public Powercell(double x, double y, double r, double theta) {
            kX = x;
            kY = y;
            kR = r;
            kTheta = theta;
        }
    }
}