package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.subsystems.FixedHoodedShooterBase;
import org.frc5587.lib.utils.UniversalEncoder;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends FixedHoodedShooterBase {
    public Shooter() {
        this(new CANSparkMax(ShooterConstants.MOTOR_ONE, MotorType.kBrushless), new CANSparkMax(ShooterConstants.MOTOR_TWO, MotorType.kBrushless));
    }

    public Shooter(CANSparkMax leaderMotor, CANSparkMax followerMotor) {
        super(new CANSparkMax[]{leaderMotor, followerMotor}, UniversalEncoder.fromCANEncoder(leaderMotor.getEncoder()));

        ShooterConstants.SHOOTER_CONTROLLER.setVelocitySupplier(this::getVelocityRPS);
        setShooterController(ShooterConstants.SHOOTER_CONTROLLER);
    }

    @Override
    protected double getVelocityRPS() {
        return encoder.getVelocity() / 60;
    }

    @Override
    protected void configureMotorControllers() {
        CANSparkMax[] motors = (CANSparkMax[]) this.motors;

        motors[0].restoreFactoryDefaults();
        motors[0].setInverted(false);
        motors[0].setIdleMode(IdleMode.kCoast);
        motors[0].setSmartCurrentLimit(40, 35);

        motors[1].restoreFactoryDefaults();
        motors[1].setInverted(false);
        motors[1].setIdleMode(IdleMode.kCoast);
        motors[1].setSmartCurrentLimit(40, 35);
    }
}