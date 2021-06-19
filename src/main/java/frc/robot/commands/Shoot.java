package frc.robot.commands;

import org.frc5587.lib.advanced.ObjectTracker;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Conveyor conveyor;
    private Intake intake;
    private LimelightCentering limelightCentering;
    private Drivetrain drivetrain;
    private ObjectTracker powerPortTracker = new ObjectTracker();

    private static double defaultDistance = 4.8;
    
    public Shoot(Shooter shooter, Limelight limelight, Conveyor conveyor, Intake intake, LimelightCentering limelightCentering, Drivetrain drivetrain) {
        super();

        this.shooter = shooter;
        this.limelight = limelight;
        this.conveyor = conveyor;
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.limelightCentering = limelightCentering;
        
        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        limelight.turnOn();
        shooter.enableDistanceControl();

        if (powerPortTracker.isReady()) {
            limelightCentering.setAngle(powerPortTracker.getRelativeAngle());
        }
        // limelightCentering.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        // limelight.turnOff();
        shooter.disableDistanceControl();
        conveyor.stopShooterConveyor();
        intake.stop();
        // limelightCentering.cancel();
    }

    public void updateShooter() {
        double distance = limelight.getDistanceFromInner();

        shooter.setDistanceFromTarget(distance);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        
        powerPortTracker.setRobotPosition(robotPose.getX(), robotPose.getY());
        if (limelight.isTargetDetected()) {
            powerPortTracker.setObjectRelativePosition(limelight.getDistanceFromInner(), Math.toRadians(robotPose.getRotation().getDegrees() - limelight.getHorizontalAngle()));

            updateShooter();
        } else {
            shooter.setDistanceFromTarget(defaultDistance);
        }

        if (shooter.atSetpoint()) {
            conveyor.shooterConveyor();
            intake.moveForward();
        }
    }
}