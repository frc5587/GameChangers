package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePistons;

public class IntakeForward extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;
    private IntakePistons intakePistons;

    public IntakeForward(Intake intake, IntakePistons intakePistons, Conveyor conveyor) {
        this.intake = intake;
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
        intake.moveForward();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stopIntakeConveyor();
    }
}