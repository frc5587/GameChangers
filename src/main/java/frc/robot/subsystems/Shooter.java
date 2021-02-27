package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax motorOne = new CANSparkMax(ShooterConstants.MOTOR_ONE, MotorType.kBrushless);
    private final CANSparkMax motorTwo = new CANSparkMax(ShooterConstants.MOTOR_TWO, MotorType.kBrushless);

    private final CANEncoder encoderOne = motorOne.getEncoder();
    private final CANEncoder encoderTwo = motorTwo.getEncoder();

    private final ShooterFeedbackController motorFeedbackController = new ShooterFeedbackController(ShooterConstants.SHOOTER_JRAD, this::getLeaderShooterSpeed);

    private double setpointVelocity = 0;
    private boolean enabled = true;
    private double lastVelocity = 0;
    private double nowVelocity = 0;

    public Shooter() {
        super();

        configureSpark();
    }

    private void configureSpark() {
        motorOne.restoreFactoryDefaults();
        motorTwo.restoreFactoryDefaults();

        motorOne.setInverted(true);
        motorTwo.setInverted(true);

        motorOne.setIdleMode(IdleMode.kCoast);
        motorTwo.setIdleMode(IdleMode.kCoast);

        motorOne.setSmartCurrentLimit(40, 35);
        motorTwo.setSmartCurrentLimit(40, 35);

        motorTwo.follow(motorTwo);
    }

    /**
     * If the JRAD control in enabled, it will update the motors based on the
     * setpoint
     */
    @Override
    public void periodic() {
        lastVelocity = nowVelocity;
        nowVelocity = encoderOne.getVelocity();
        // log();
        motorFeedbackController.sendDebugInfo();
        // motorTwoController.sendDebugInfo();

        if (enabled) {
            setVoltage(motorFeedbackController.setSetpointAndCalculate(setpointVelocity));
        }
    }

    /**
     * Returns that average motor speed between the two shooter motors
     * 
     * @return average velocity - ROTATIONS PER SECOND
     */
    public double getAvgShooterSpeed() {
        return (encoderOne.getVelocity() + encoderTwo.getVelocity()) / 2;
    }

    /**
     * Returns the velocity of the leader motor
     * 
     * @return velocity - ROTATIONS PER SECOND
     */
    public double getLeaderShooterSpeed() {
        return encoderOne.getVelocity();
    }

    /**
     * Returns the velocity of the follower motor
     * 
     * @return velocity - ROTATIONS PER SECOND
     */
    public double getFollowerShooterSpeed() {
        return encoderTwo.getVelocity();
    }

    public void setVoltage(double voltage) {
        motorOne.setVoltage(voltage);
    }

    /**
     * Gets the velocity of the first motor
     * 
     * @return velocity - ROTATIONS PER SECOND
     */
    public double getMotorOneVelocity() {
        return encoderOne.getVelocity();
    }

    /**
     * Gets the velocity of the second motor
     * 
     * @return velocity - ROTATIONS PER SECOND
     */
    public double getMotorTwoVelocity() {
        return encoderTwo.getVelocity();
    }

    /**
     * Enables JRAD control
     */
    public void enableJRAD() {
        motorFeedbackController.reset();
        enabled = true;
    }

    /**
     * Disables JRAD control and sets throttle to 0
     */
    public void disableJRAD() {
        enabled = false;
        setThrottle(0);
    }

    /**
     * Sets a new velocity setpoint for the shooter
     * 
     * @param setpointVelocity new setpoint - ROTATIONS PER SECOND
     */
    public void setVelocity(double setpointVelocity) {
        this.setpointVelocity = setpointVelocity;
    }

    /**
     * If JRAD is disable, this controls the percent output of the shooter
     * 
     * @param throttle percent output - [-1, 1]
     */
    public void setThrottle(double throttle) {
        if (!enabled) {
            motorOne.set(throttle);
        }
    }

    public boolean atSetpoint() {
        return motorFeedbackController.atSetpoint();
    }

}