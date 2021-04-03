package frc.robot.commands;

import org.frc5587.lib.pid.PIDTunerHelper;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePistons;

public class IntakeForward extends CommandBase {
    private Intake intake;
    private Drivetrain drivetrain;
    private Conveyor conveyor;
    private IntakePistons intakePistons;
    // private final PIDTunerHelper tuner = new PIDTunerHelper("intake", IntakeConstants.PID.kP, IntakeConstants.PID.kI, IntakeConstants.PID.kD);

    public IntakeForward(Intake intake, IntakePistons intakePistons, Drivetrain drivetrain, Conveyor conveyor) {
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;

        addRequirements(intake, intakePistons, conveyor);
    }

    @Override
    public void initialize() {
        intakePistons.extend();
        conveyor.intakeConveyor();
    }

    /**
     * Gives the PID controller either twice the linear speed of the robot, or
     * MIN_THROTTLE, whichever is larger. This will prevent the intake from
     * essentially not moving if the robot is stationary
     */
    @Override
    public void execute() {
        // intake.set(controller.calculate(intake.getSurfaceSpeedMetersPerSecond(), Math.max(IntakeConstants.MIN_THROTTLE,
        //         drivetrain.getAbsoluteAverageVelocityMetersPerSecond() * IntakeConstants.VELOCITY_MULTIPLIER)));
        // intake.set(tuner.calculate(intake.getSurfaceSpeedMetersPerSecond()));
        // intake.set(1);
        intake.moveForward();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stopIntakeConveyor();
    }
}