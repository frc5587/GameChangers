package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    private final CANSparkMax leftLeader = new CANSparkMax(DrivetrainConstants.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(DrivetrainConstants.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

    private final DifferentialDrive differentialDrive;

    /**
     * Creates a new Drive.
     */

    public Drivetrain() {
        leftLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        var leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
        var rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

        leftGroup.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightGroup.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);

        leftLeader.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
        leftFollower.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
        rightLeader.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
        rightFollower.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);

        leftLeader.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
        leftFollower.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
        rightLeader.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
        rightFollower.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);

        this.differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    }

    public void arcadeDrive(double throttle, double curve) {
        differentialDrive.arcadeDrive(throttle, curve);
    }

    public void tankLR(double leftThrottle, double rightThrottle) {
        differentialDrive.tankDrive(leftThrottle, rightThrottle, false);
    }

    public void tankLRVolts(double leftVoltage, double rightVoltage) {
        // Convert voltages to percents by dividing by maximum value
        tankLR(leftVoltage / 12.0, rightVoltage / 12.0);
    }

    public void stop() {
        differentialDrive.stopMotor();
    }

}
