package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PowercellDetector;

public class MoveToPowercell extends CommandBase {
    private PowercellDetector powercellDetector;
    private Drivetrain drivetrain;
    private RamseteCommandWrapper ramseteCommand;
    private IntakeForward intakeForward;

    public MoveToPowercell(PowercellDetector powercellDetector, Drivetrain drivetrain, IntakeForward intakeForward) {
        this.drivetrain = drivetrain;
        this.powercellDetector = powercellDetector;
        this.intakeForward = intakeForward;

        addRequirements(powercellDetector, drivetrain);
    }

    @Override
    public void initialize() {

        if (powercellDetector.seesPowercell()) {
            double powercellX = powercellDetector.getPowercellX();
            double powercellY = powercellDetector.getPowercellY();
            double angle = powercellDetector.getHorizontalAngleRadians();
            
            ramseteCommand = new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(powercellX, -powercellY, new Rotation2d(-angle)));
            ramseteCommand.schedule();
            intakeForward.schedule();
        } else {
            ramseteCommand = null;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (ramseteCommand != null) {   
            ramseteCommand.cancel();
            ramseteCommand = null;

            intakeForward.cancel();
        }
    }
}