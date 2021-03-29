package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR,
            MotorType.kBrushless);

    /**
     * Creates a new Intake.
     */
    public Intake() {
        super();

        configureSpark();
    }

    private void configureSpark() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(true);

        intakeMotor.setIdleMode(IdleMode.kBrake);
    }
    /**
     * Move the intake forward
     */
    public void moveForward() {
        intakeMotor.set(Constants.IntakeConstants.THROTTLE);
    }

    /**
     * Move the intake backward
     */
    public void moveBackward() {
        intakeMotor.set(-Constants.IntakeConstants.THROTTLE);
    }

    /**
     * Stop the intake
     */
    public void stop() {
        intakeMotor.set(0);
    }
}
