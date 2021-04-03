package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakePistons extends SubsystemBase {
    private DoubleSolenoid piston1 = new DoubleSolenoid(IntakeConstants.PISTON_PORTS[0], IntakeConstants.PISTON_PORTS[1]);

    public IntakePistons() {
        retract(); // idk what this will do but I'm just trying to set the default value to closed
    }

    public void extend() {
        piston1.set(kForward);
    }

    public void retract() {
        piston1.set(kReverse);
    }
}