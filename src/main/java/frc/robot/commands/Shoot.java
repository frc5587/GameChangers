package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;

    private static double defaultSpinUpSpeedRPS = 16;  // RPM
    
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
        shooter.enableJRADControl();
    }

    @Override
    public void end(boolean interrupted) {
        limelight.turnOff();
        shooter.disableJRADControl();
    }

    public void updateShooter() {
        double distance = limelight.getDistanceFromInner();

        shooter.calculateAndSetVelocity(distance);
    }

    @Override
    public void execute() {
        if (limelight.isTargetDetected()) {
            updateShooter();
        } else {
            shooter.setVelocityRPS(defaultSpinUpSpeedRPS);
        }
    }
}