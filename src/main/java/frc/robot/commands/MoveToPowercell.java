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

    public MoveToPowercell(PowercellDetector powercellDetector, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.powercellDetector = powercellDetector;

        addRequirements(powercellDetector, drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println(powercellDetector.seesPowercell());

        if (powercellDetector.seesPowercell()) {
            double powercellX = powercellDetector.getPowercellX();
            double powercellY = powercellDetector.getPowercellY();
            double angle = powercellDetector.getHorizontalAngleRadians();
            
            ramseteCommand = new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(powercellX, powercellY, new Rotation2d(angle)));
            ramseteCommand.schedule();
        } else {
            ramseteCommand = null;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (ramseteCommand != null) {
            ramseteCommand.end(interrupted);
            ramseteCommand = null;
        }
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