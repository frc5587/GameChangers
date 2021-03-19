/**
 * THIS IS A STUB
 * 
 * This is just to get the MoveToPowercell working 
 */


package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Drivetrain extends PIDSubsystem {

    public Drivetrain() {
        super(null);
    }

    public void logData() {
    }

    public void arcadeDrive(double throttle, double curve) {
    }

    public void tankLR(double leftThrottle, double rightThrottle) {
    }

    public void tankLRVolts(double leftVolts, double rightVolts) {
    }

    public void stop() {
    }

    public double getLeftPositionRotations() {
        return 0;
    }

    public double getRightPositionRotations() {
        return 0;
    }

    public double getLeftPositionMeters() {
        return 0;
    }

    public double getRightPositionMeters() {
        return 0;
    }

    public double getRightVelocityRotationsPerMinute() {
        return 0;
    }

    public double getLeftVelocityRotationsPerMinute() {
        return 0;
    }

    private double rotationsPerMinuteToMetersPerSecond(double rotationsPerMinute) {
        return 0;
    }

    public double getLeftVelocityMetersPerSecond() {
        return 0;
    }

    public double getRightVelocityMetersPerSecond() {
        return 0;
    }

    /**
     * Get the raw, unbounded heading of the drivetrain's gyroscope in degrees. To
     * get the bounded gyro scope heading between -180 and +180, use
     * {@link #getHeading180()} instead.
     * 
     * @return the raw, unbounded heading of the drivetrain gyro in degrees
     */
    public double getHeading() {
        return 0;
    }

    public double getHeading360() {
        return 0;
    }

    /**
     * Get the heading of the drivetrain's gyroscope, wrapped to be within -180 to
     * +180. With these bounds, 0 degrees is still the angle that the drivetrain
     * started with when the robot was turned on.
     * 
     * @returns the heading in degrees, where -180 < heading < +180
     */
    public double getHeading180() {
        return 0;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
    }

    @Override
    public double getMeasurement() {
        return 0;
    }

    public Pose2d getPose() {
        return null;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return null;
    }

    public Pose2d getClosestPoseAtTime(double time) {
        return null;
    }

    public void resetEncoders() {
    }

    public void resetHeading() {
    }

    public void resetOdometry() {
    }

    // TODO: does this work?
    public void resetOdometry(Pose2d pose) {
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return 0;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return 0;
    }

    public void setIdleMode(IdleMode idleMode) {
    }

    @Override
    public void setSetpoint(double setpoint) {
    }

    /**
     * Whether the angle PID controller is currently at its last setpoint. This does
     * not check whether the PID controller itself is enabled.
     * 
     * @return whether the angle PID controller is within the angle tolerance
     */
    public boolean atSetpoint() {
        return false;
    }

    /**
     * @return the last angle setpoint for the turn PID controller
     */
    public double getLastAngleSetpoint() {
        return 0;
    }

    @Override
    public void periodic() {
    }

    public void stopDrivetrain() {
    }

    public void setDrive(double speed) {
    }

}
