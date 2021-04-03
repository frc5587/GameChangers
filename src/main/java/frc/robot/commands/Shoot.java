package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants.RegressionConstants;

public class Shoot extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Conveyor conveyor;
    private boolean active = false;

    private static double defaultSpinUpSpeed = 3000;  // RPM
    
    public Shoot(Shooter shooter, Limelight limelight, Conveyor conveyor) {
        super();

        this.shooter = shooter;
        this.limelight = limelight;
        this.conveyor = conveyor;
        
        addRequirements(shooter, conveyor);
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
        conveyor.stopShooterConveyor();
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

            if (shooter.atSetpoint()) {
                conveyor.shooterConveyor();
            }
        }
    }
}