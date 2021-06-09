package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Conveyor conveyor;
    private Intake intake;
    private LimelightCentering limelightCentering;

    private static double defaultDistance = 4.8;
    
    public Shoot(Shooter shooter, Limelight limelight, Conveyor conveyor, Intake intake, LimelightCentering limelightCentering) {
        super();

        this.shooter = shooter;
        this.limelight = limelight;
        this.conveyor = conveyor;
        this.intake = intake;
        this.limelightCentering = limelightCentering;
        
        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        limelight.turnOn();
        shooter.enableDistanceControl();
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
        if (limelight.isTargetDetected()) {
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