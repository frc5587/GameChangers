package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class IntakeForward extends CommandBase {
    private Intake intake;
    private Drivetrain drivetrain;
    private final PIDController controller = new PIDController(IntakeConstants.PID.kP, IntakeConstants.PID.kI,
            IntakeConstants.PID.kD);

    public IntakeForward(Intake intake, Drivetrain drivetrain) {
        this.intake = intake;
        this.drivetrain = drivetrain;

        addRequirements(intake);
    }

    /**
     * Gives the PID controller either twice the linear speed of the robot, or
     * MIN_THROTTLE, whichever is larger. This will prevent the intake from
     * essentially not moving if the robot is stationary
     */
    @Override
    public void execute() {
        intake.set(controller.calculate(intake.getSurfaceSpeedMetersPerSecond(), Math.max(IntakeConstants.MIN_THROTTLE,
                drivetrain.getAbsoluteAverageVelocityMetersPerSecond() * IntakeConstants.VELOCITY_MULTIPLIER)));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}