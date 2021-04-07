package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.LimitedPoseMap;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends PIDSubsystem {
    private final CANSparkMax leftLeader = new CANSparkMax(DrivetrainConstants.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(DrivetrainConstants.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

    private final CANEncoder leftEncoder = leftLeader.getAlternateEncoder(AlternateEncoderType.kQuadrature,
            DrivetrainConstants.TICKS_PER_REV);
    private final CANEncoder rightEncoder = rightLeader.getAlternateEncoder(AlternateEncoderType.kQuadrature,
            DrivetrainConstants.TICKS_PER_REV);
    private final AHRS ahrs = new AHRS();

    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    private final DifferentialDriveOdometry odometry;

    private final LimitedPoseMap poseHistory = new LimitedPoseMap(DrivetrainConstants.HISTORY_LIMIT);

    private final PIDController turnController = getController();
    private double lastAngleSetpoint = Double.NaN;

    /**
     * Creates a new Drive.
     */
    public Drivetrain() {
        super(
                // The PIDController used for auto-centering the drivetrain
                new PIDController(DrivetrainConstants.TURN_FPID.kP, DrivetrainConstants.TURN_FPID.kI,
                        DrivetrainConstants.TURN_FPID.kD));

        configureSpark();

        var currentAngle = Rotation2d.fromDegrees(getHeading360());
        this.odometry = new DifferentialDriveOdometry(currentAngle);
        // TODO: Allow for selecting of several initial positions with odometry, instead
        // of assuming x=0, y=0

        // Configure turn PID
        this.disable();
        // var controller = this.getController();
        turnController.enableContinuousInput(-180, 180);
        turnController.setIntegratorRange(-1, 1);
        turnController.setTolerance(DrivetrainConstants.TURN_PID_TOLERANCE_DEG);
    }

    /**
     * Configures SparkMaxes
     */
    private void configureSpark() {
        leftLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();
        
        setIdleMode(IdleMode.kBrake);

        leftGroup.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightGroup.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
        leftEncoder.setInverted(DrivetrainConstants.LEFT_ENCODER_INVERTED);
        rightEncoder.setInverted(DrivetrainConstants.RIGHT_ENCODER_INVERTED);

        leftLeader.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
        leftFollower.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
        rightLeader.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
        rightFollower.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);

        leftLeader.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
        leftFollower.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
        rightLeader.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
        rightFollower.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
    }

    /**
     * Takes drivetrain data and inputs it into SmartDashboard
     */
    public void logData() {
        SmartDashboard.putNumber("left enc", leftEncoder.getPosition());
        SmartDashboard.putNumber("right enc", rightEncoder.getPosition());
        SmartDashboard.putNumber("gyro", ahrs.getAngle());
        SmartDashboard.putNumber("thing that max wants (x)", ahrs.getDisplacementX());
        SmartDashboard.putNumber("thing that max wants (y)", ahrs.getDisplacementY());
        // System.out.println("ahrs: " + ahrs.getAngle());
        // System.out.println("lv: " + leftEncoder.getPosition() + " rv: " + rightEncoder.getPosition());
    }

    /**
     * Controls the drvivetrain through a differential drive with throttle and curve values
     * @param throttle
     * @param curve
     */
    public void arcadeDrive(double throttle, double curve) {
        differentialDrive.arcadeDrive(throttle, curve, false);
    }

    /**
     * Controls the drivetrain through left and right throttle values
     * @param leftThrottle
     * @param rightThrottle
     */
    public void tankLR(double leftThrottle, double rightThrottle) {
        differentialDrive.tankDrive(leftThrottle, rightThrottle, false);
    }

    /**
     * Controls the drivetrain by setting lef tand right volt output
     * @param leftVolts
     * @param rightVolts
     */
    public void tankLRVolts(double leftVolts, double rightVolts) {
        // Convert voltages to percents by dividing by maximum value
        // tankLR(-leftVoltage / 12.0, -rightVoltage / 12.0);
        leftGroup.setVoltage(-leftVolts);
        rightGroup.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    /**
     * Stops the drivetrain
     */
    public void stop() {
        differentialDrive.stopMotor();
    }

    /**
     * Gets position of the left side using encoders
     * @return position of the left side of the robot
     */
    public double getLeftPositionRotations() {
        return leftEncoder.getPosition() / 2;
    }

    /**
     * Gets position of the right side using encoders
     * @return position of the right side of the robot
     */
    public double getRightPositionRotations() {
        return rightEncoder.getPosition() / 2;
    }

    /**
     * Takes number of rotations wheel has gone through and converts it to meters
     * @param rotations rotations which the wheels have gone through
     * @return distance robot has travelled in meters
     */
    private double rotationsToMeters(double rotations) {
        // number of rotations * circumference of wheel
        return rotations * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI;
    }

    /**
     * @return how far the left side of the robot has travelled
     */
    public double getLeftPositionMeters() {
        return rotationsToMeters(getLeftPositionRotations());
    }

    /**
     * @return how far the right side of the robot has travelled
     */
    public double getRightPositionMeters() {
        return rotationsToMeters(getRightPositionRotations());
    }

    /**
     * @return the velocity of the right side in rotations per minute
     */
    public double getRightVelocityRotationsPerMinute() {
        return rightEncoder.getVelocity() / 2;
    }

    /**
     * @return the velocity of the left side in rotations per minute
     */
    public double getLeftVelocityRotationsPerMinute() {
        return leftEncoder.getVelocity() / 2;
    }

    /**
     * Converts rotational velocity to linear velocity
     * @param rotationsPerMinute
     * @return linear velocity in meters per second
     */
    private double rotationsPerMinuteToMetersPerSecond(double rotationsPerMinute) {
        var radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rotationsPerMinute);
        var linearMetersPerSecond = radiansPerSecond * DrivetrainConstants.WHEEL_RADIUS_METERS;
        return linearMetersPerSecond;
    }

    /**
     * @return velocity of the left side in meters per second
     */
    public double getLeftVelocityMetersPerSecond() {
        return rotationsPerMinuteToMetersPerSecond(getLeftVelocityRotationsPerMinute());
    }

    /**
     * @return velocity of the right side in meters per second
     */
    public double getRightVelocityMetersPerSecond() {
        return rotationsPerMinuteToMetersPerSecond(getRightVelocityRotationsPerMinute());
    }

    public double getAbsoluteAverageVelocityMetersPerSecond() {
        return (Math.abs(getRightVelocityMetersPerSecond()) + Math.abs(getLeftVelocityMetersPerSecond())) / 2;
    }

    /**
     * Get the raw, unbounded heading of the drivetrain's gyroscope in degrees. To
     * get the bounded gyro scope heading between -180 and +180, use
     * {@link #getHeading180()} instead.
     * 
     * @return the raw, unbounded heading of the drivetrain gyro in degrees
     */
    public double getHeading() {
        return ahrs.getAngle() * (DrivetrainConstants.INVERT_GYRO_DIRECTION ? -1 : 1);
        // return gyro.getAngle();
    }
    
    public double getHeading360() {
        return Math.IEEEremainder(getHeading(), 360.0/* d */);
        // return gyro.getAngle();
    }

    /**
     * Get the heading of the drivetrain's gyroscope, wrapped to be within -180 to
     * +180. With these bounds, 0 degrees is still the angle that the drivetrain
     * started with when the robot was turned on.
     * 
     * @returns the heading in degrees, where -180 < heading < +180
     */
    public double getHeading180() {
        var heading = getHeading() % 360;
        if (heading > 180) {
            return heading - 360;
        } else if (heading < -180) {
            return heading + 360;
        } else {
            return heading;
        }
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // Use the output and add the turn PID feed forward constant
        arcadeDrive(DrivetrainConstants.TURN_PID_FORWARD_THROTTLE,
                output + Math.copySign(DrivetrainConstants.TURN_FPID.kF, output));
    }

    @Override
    public double getMeasurement() {
        // Return the process variable measurement here
        return getHeading();
    }

    public Pose2d getPose() {
        var val = odometry.getPoseMeters(); 
        System.out.println("" + val.getX() + "  " +val.getY());
        return val;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    public Pose2d getClosestPoseAtTime(double time) {
        return poseHistory.getClosest(time);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetHeading() {
        ahrs.reset();
    }

    public void resetOdometry() {
        resetHeading();
        resetEncoders();
        odometry.resetPosition(new Pose2d(), new Rotation2d());
    }

    /**
     * resets the entire odometry and encoders
     * @param pose pose of the robot at a certain instance
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, ahrs.getRotation2d());
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -ahrs.getRate();
    }

    /** 
     * Sets idle mode of all motors
    */
    public void setIdleMode(IdleMode idleMode) {
        leftLeader.setIdleMode(idleMode);
        leftFollower.setIdleMode(idleMode);
        rightLeader.setIdleMode(idleMode);
        rightFollower.setIdleMode(idleMode);
    }

    @Override
    public void setSetpoint(double setpoint) {
        lastAngleSetpoint = setpoint;
        super.setSetpoint(setpoint);
    }

    /**
     * Whether the angle PID controller is currently at its last setpoint. This does
     * not check whether the PID controller itself is enabled.
     * 
     * @return whether the angle PID controller is within the angle tolerance
     */
    public boolean atSetpoint() {
        return turnController.atSetpoint();
    }

    /**
     * @return the last angle setpoint for the turn PID controller
     */
    public double getLastAngleSetpoint() {
        return lastAngleSetpoint;
    }

    @Override
    public void periodic() {
        // Call periodic of PIDSubsystem to ensure PID controller runs
        super.periodic();
        logData();

        // Update the pose
        var gyroAngle = Rotation2d.fromDegrees(getHeading360());
        odometry.update(gyroAngle, getLeftPositionMeters(), getRightPositionMeters());
        var translation = odometry.getPoseMeters().getTranslation();
        SmartDashboard.putNumber("tx", translation.getX());
        SmartDashboard.putNumber("ty", translation.getY());
        // SmartDashboard.putNumber("angle", odo

        // Log the pose
        poseHistory.put(Timer.getFPGATimestamp(), getPose());
    }

    /**
     * stops the drivetrain without using differential drive
     */
    public void stopDrivetrain() {
        leftGroup.set(0);
        rightGroup.set(0);
    }

    /**
     * sets drivetrain speed without using differential drive
     * @param speed desired speed of motors, from -1 to 1
     */
    public void setDrive(double speed) {
        leftGroup.set(-speed);
        rightGroup.set(speed);
    }

}
