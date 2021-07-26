package frc.robot.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR,
            MotorType.kBrushless);
    private final CANEncoder intakeEncoder = intakeMotor.getEncoder();

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
        intakeMotor.setSmartCurrentLimit(35, 40);

        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getSurfaceSpeedMetersPerSecond() {
        return Units.rotationsPerMinuteToRadiansPerSecond(intakeEncoder.getVelocity()) * IntakeConstants.INTAKE_RADIUS_METERS;
    }

    public void set(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Move the intake forward
     */
    public void moveForward() {
        intakeMotor.set(IntakeConstants.THROTTLE);
    }

    /**
     * Move the intake backward
     */
    public void moveBackward() {
        intakeMotor.set(-IntakeConstants.THROTTLE);
    }

    /**
     * Stop the intake
     */
    public void stop() {
        intakeMotor.set(0);
    }
}
