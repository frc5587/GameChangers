package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PowercellDetector;
import frc.robot.subsystems.PowercellDetector.Powercell;

public class MoveToAllPowercells extends CommandBase {
    private PowercellDetector powercellDetector;
    private Drivetrain drivetrain;
    private RamseteCommandWrapper ramseteCommand;
    private IntakeForward intakeForward;
    private Pose2d finalPosition;

    public MoveToAllPowercells(PowercellDetector powercellDetector, Drivetrain drivetrain, IntakeForward intakeForward) {
        this.drivetrain = drivetrain;
        this.powercellDetector = powercellDetector;
        this.intakeForward = intakeForward;

        addRequirements(powercellDetector, drivetrain);
    }

    @Override
    public void initialize() {
        if (powercellDetector.seesPowercell()) {
            List<Powercell> powercells = powercellDetector.getAllPowercells();

            if (powercells.size() == 1) {  // if only one powercell is seen, it skips the waypoints
                Powercell powercell = powercells.get(0);
                ramseteCommand = new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(powercell.kX, -powercell.kY, new Rotation2d(-powercell.kTheta)));
            } else if (finalPosition != null) { // if finalPosition is set, all powercells become waypoints
                List<Translation2d> waypoints = Arrays.asList();

                for (Powercell pc : powercells) {
                    waypoints.add(new Translation2d(pc.kX, -pc.kY));
                }

                ramseteCommand = new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), waypoints, finalPosition);
                finalPosition = null;
            } else {  // the final powercell is the destination, and the other powercells become waypoints
                Powercell powercell = powercells.remove(powercells.size() - 1);
                List<Translation2d> waypoints = Arrays.asList();

                for (Powercell pc : powercells) {
                    waypoints.add(new Translation2d(pc.kX, -pc.kY));
                }

                ramseteCommand = new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), waypoints, new Pose2d(powercell.kX, -powercell.kY, new Rotation2d(-powercell.kTheta)));
            }
            
            ramseteCommand.schedule();
            intakeForward.schedule();
        } else {
            ramseteCommand = null;
        }
    }

    public MoveToAllPowercells endAt(Pose2d pos) {
        finalPosition = pos;
        return this;
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