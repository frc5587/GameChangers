package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final CANSparkMax climberMotor = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR, MotorType.kBrushless);

    /**
     * Creates new Climber
     */
    public Climber() {
        configureSpark();
    }

    private void configureSpark() {
        climberMotor.restoreFactoryDefaults();

        climberMotor.setInverted(false);
        climberMotor.setSmartCurrentLimit(35, 40);

        climberMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Move the climber up
     */
    public void moveUp() {
        climberMotor.set(ClimberConstants.CLIMBER_THROTTLE);
    }

    /**
     * Move climber down
     */
    public void moveDown() {
        climberMotor.set(-ClimberConstants.CLIMBER_THROTTLE);
    }

    /**
     * Stop climber
     */
    public void stopClimber() {
        climberMotor.set(0);
    }
}
    
