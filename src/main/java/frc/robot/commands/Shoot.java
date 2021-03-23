package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants.RegressionConstants;

public class Shoot extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private boolean active = false;

    private static double defaultSpinUpSpeed = 1000;  // RPM
    
    public Shoot(Shooter shooter, Limelight limelight) {
        super();

        this.shooter = shooter;
        this.limelight = limelight;
        
        
        addRequirements(shooter, limelight);
        SmartDashboard.putNumber("try velocity RPM", 0);
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

        double wheelRPM = (RegressionConstants.U * distance) + (RegressionConstants.P/(Math.pow(distance, 2) - RegressionConstants.N)); 

        shooter.setVelocity(wheelRPM);
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