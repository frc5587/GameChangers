package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class Shoot extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private boolean active = false;

    private static double defaultSpinUpSpeed = 10;  // RPS
    
    public Shoot(Shooter shooter, Limelight limelight) {
        super();

        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(shooter, limelight);
    }

    @Override
    public void initialize() {
        limelight.turnOn();
        shooter.enableJRAD();

        active = true;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.turnOff();
        shooter.disableJRAD();
    }

    public void updateShooter() {
        double distance = limelight.getDistanceFromInner();
        double height = ShooterConstants.GOAL_HEIGHT - ShooterConstants.SHOOTER_HEIGHT;

        double ballVelocity = Math.sqrt(Math.pow(distance, 2) * ShooterConstants.G / (2 * Math.cos(ShooterConstants.SHOOTER_ANGLE) * (height - (distance * Math.tan(ShooterConstants.SHOOTER_ANGLE)))));
        double wheelRPS = ballVelocity / (2 * Math.PI * ShooterConstants.WHEEL_RADIUS);

        shooter.setVelocity(wheelRPS);
    }

    @Override
    public void execute() {
        if (active) {
            if (limelight.isTargetDetected()) {
                updateShooter();
            } else {
                shooter.setVelocity(defaultSpinUpSpeed);
            }
        }
    }
}