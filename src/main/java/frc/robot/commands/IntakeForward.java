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
        intake.moveForward();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stopIntakeConveyor();
    }
}