package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.LimelightConstants;

/**
 * The subsystem for the Limelight
 */
public class Limelight extends SubsystemBase {
    public NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    public NetworkTableEntry tv = limelightTable.getEntry("tv");
    public NetworkTableEntry tx = limelightTable.getEntry("tx");
    public NetworkTableEntry ty = limelightTable.getEntry("ty");
    public double lastDistance;
    private boolean lightOn = false;

    public Limelight() {
        super();
        turnOff();
    }

    /**
     * Get whether the target is being detected by the Limelight
     * 
     * @return whether the target is being detected
     */
    public boolean isTargetDetected() {
        return tv.getDouble(0) == 1;
    }

    public double getRelativeVerticalAngle() {
        return Math.toRadians(ty.getDouble(0));
    }

    /**
     * Gets the vertical angle between the limelight and the outer target
     * 
     * @return
     */
    public double getVerticalAngleToOuter() {
        return LimelightConstants.LIMELIGHT_ANGLE + getRelativeVerticalAngle();
    }

    public double getDistanceFromOuter() {
        return (LimelightConstants.GOAL_HEIGHT_METERS - LimelightConstants.LIMELIGHT_HEIGHT) 
            / Math.tan(getVerticalAngleToOuter());
    }

    public double getDistanceFromInner() {
        return getDistanceFromOuter() + LimelightConstants.INNER_OUTER_GOAL_DISTANCE_METERS;
    }


    @Override
    public void periodic() {
        if (lightOn) {
            limelightTable.getEntry("ledMode").setNumber(3);
        } else {
            limelightTable.getEntry("ledMode").setNumber(1);
        }
    }

    public void turnOn() {
        lightOn = true;
    }

    public void turnOff() {
        lightOn = false;
    }

}