package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;

/**
 * The subsystem for the Limelight
 */
public class Limelight extends SubsystemBase {
    public NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    public NetworkTableEntry tv = limelightTable.getEntry("tv");
    public NetworkTableEntry tx = limelightTable.getEntry("tx");
    public NetworkTableEntry ty = limelightTable.getEntry("ty");
    private NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");
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
    public double getVerticalAngle() {
        return LimelightConstants.LIMELIGHT_ANGLE + getRelativeVerticalAngle();
    }

    public double getHorizontalAngle() {
        return tx.getDouble(0);
    }

    public double getDistanceFromOuter() {
        return (LimelightConstants.GOAL_HEIGHT - LimelightConstants.LIMELIGHT_HEIGHT - LimelightConstants.VERTICAL_GOAL_OFFSET) 
            / Math.tan(getVerticalAngle());
    }

    public double getDistanceFromInner() {
        return getDistanceFromOuter() + LimelightConstants.INNER_OUTER_GOAL_DISTANCE;
    }

    @Override
    public void periodic() {
        ledMode.setNumber(lightOn? 3 : 1);

        if (isTargetDetected()) {
            SmartDashboard.putNumber("distance", getDistanceFromOuter());
        }
    }

    public void turnOn() {
        lightOn = true;
    }

    public void turnOff() {
        lightOn = false;
    }

}