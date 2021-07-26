package frc.robot.commands;

import org.frc5587.lib.advanced.ObjectTracker;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
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

    private Timer conveyorTimer = new Timer();
    private Timer otherConveyorTimer = new Timer();
    private static double defaultDistance = 4.8;
    private static double backwardsConveyorTime = .15;
    private static double backDelay = .15;

    public Shoot(Shooter shooter, Limelight limelight, Conveyor conveyor, Intake intake,
            LimelightCentering limelightCentering, Drivetrain drivetrain) {
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
        conveyorTimer.reset();
        conveyorTimer.start();
        conveyor.reverse();
        limelight.turnOn();
        shooter.enableDistanceControl();

        if (powerPortTracker.isReady()) {
            limelightCentering.setAngle(powerPortTracker.getRelativeAngle());
        }
        // limelightCentering.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        limelight.turnOff();
        shooter.disableDistanceControl();
        conveyor.stopShooterConveyor();
        intake.stop();
        // limelightCentering.cancel();

        otherConveyorTimer.stop();
        otherConveyorTimer.reset();
    }

    public void updateShooter() {
        double distance = limelight.getDistanceFromInner();
        shooter.setDistanceFromTarget(distance);

        // if (distance > 6.5) {
        //     shooter.setDistanceFromTarget(.90 * distance);
        // } else if (distance > 5) {
        //     shooter.setDistanceFromTarget(.85 * distance);
        // } else {
        //     shooter.setDistanceFromTarget(distance);
        // }
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();

        powerPortTracker.setRobotPosition(robotPose.getX(), robotPose.getY());

        if (limelight.isTargetDetected()) {
            powerPortTracker.setObjectRelativePosition(limelight.getDistanceFromInner(),
                    Math.toRadians(robotPose.getRotation().getDegrees() - limelight.getHorizontalAngle()));
            updateShooter();
        } else {
            shooter.setDistanceFromTarget(defaultDistance);
        }

        if (shooter.atSetpoint()) {
            otherConveyorTimer.start();
            conveyor.backConveyor();
        }

        if (otherConveyorTimer.hasElapsed(backDelay)) {
            conveyor.shooterConveyor();
            intake.moveForward();
        }

        if (conveyorTimer.hasElapsed(backwardsConveyorTime)) {
            conveyor.stopIntakeConveyor();
            conveyorTimer.stop();
            conveyorTimer.reset();
        }

        SmartDashboard.putNumber("distance from target", limelight.getDistanceFromInner());
    }
}