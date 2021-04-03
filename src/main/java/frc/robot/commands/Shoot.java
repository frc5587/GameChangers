package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static double defaultSpinUpSpeedRPS = 40;
    
    public Shoot(Shooter shooter, Limelight limelight, Conveyor conveyor, Intake intake) {
        super();

        this.shooter = shooter;
        this.limelight = limelight;
        this.conveyor = conveyor;
        this.intake = intake;
        
        addRequirements(shooter, conveyor);
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
        conveyor.stopShooterConveyor();
        intake.stop();
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

        if (shooter.atSetpoint()) {
            conveyor.shooterConveyor();
            intake.moveForward();
        }
    }
}