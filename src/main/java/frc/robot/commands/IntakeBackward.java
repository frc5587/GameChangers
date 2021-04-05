package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeBackward extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;

    public IntakeBackward(Intake intake,  Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;

        addRequirements(intake, conveyor);
    }

    @Override
    public void initialize() {
        intake.moveBackward();
        conveyor.reverse();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stopIntakeConveyor();
    }
}