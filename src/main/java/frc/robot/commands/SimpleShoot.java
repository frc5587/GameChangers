package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SimpleShoot extends CommandBase {
    private Shooter shooter;
    private DoubleSupplier velocitySupplier;

    public SimpleShoot(Shooter shooter, DoubleSupplier velocitySupplier) {
        this.shooter = shooter;
        this.velocitySupplier = velocitySupplier;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setThrottle(velocitySupplier.getAsDouble());
    }
}
