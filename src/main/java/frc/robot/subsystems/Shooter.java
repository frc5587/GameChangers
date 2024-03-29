package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import org.frc5587.lib.subsystems.FixedHoodedShooterBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends FixedHoodedShooterBase {
    public Shooter() {
        super(ShooterConstants.MOTOR_ONE, ShooterConstants.MOTOR_TWO);

        ShooterConstants.SHOOTER_CONTROLLER.setVelocitySupplier(this::getVelocityRPS);
        setShooterController(ShooterConstants.SHOOTER_CONTROLLER);
    }

    @Override
    protected double getVelocityRPS() {
        return encoder.getVelocity() / 60;
    }

    @Override
    protected void configureLeaderSpark() {
        leadMotor.restoreFactoryDefaults();
        leadMotor.setInverted(false);
        leadMotor.setIdleMode(IdleMode.kCoast);
        leadMotor.setSmartCurrentLimit(40, 35);
    }

    @Override
    protected void configureFollowerSpark() {
        followerMotor.restoreFactoryDefaults();
        followerMotor.setInverted(true);
        followerMotor.setIdleMode(IdleMode.kCoast);
        followerMotor.setSmartCurrentLimit(40, 35);
    }
}