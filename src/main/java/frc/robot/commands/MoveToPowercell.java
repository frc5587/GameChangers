package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PowercellDetector;

public class MoveToPowercell extends CommandBase {
    private PowercellDetector powercellDetector;
    private Drivetrain drivetrain;
    private RamseteCommandWrapper ramseteCommand;

    public MoveToPowercell(PowercellDetector powercellDetector, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.powercellDetector = powercellDetector;

        addRequirements(powercellDetector, drivetrain);
    }

    @Override
    public void initialize() {
        if (powercellDetector.seesPowercell()) {
            double powercellX = powercellDetector.getPowercellX();
            double powercellY = powercellDetector.getPowercellY();
            double angle = powercellDetector.getHorizontalAngleRadians();
            
            ramseteCommand = new RamseteCommandWrapper(drivetrain, powercellX, powercellY, angle);
            ramseteCommand.schedule();
        } else {
            ramseteCommand = null;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
        ramseteCommand = null;
    }

    @Override
    public boolean isFinished() {
        if (ramseteCommand == null) {
            return true;
        } else {
            return ramseteCommand.isFinished();
        }
    }
}