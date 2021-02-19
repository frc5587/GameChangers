package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final CANSparkMax motorOne = new CANSparkMax(ShooterConstants.MOTOR_ONE, MotorType.kBrushless); 
    private final CANSparkMax motorTwo = new CANSparkMax(ShooterConstants.MOTOR_TWO, MotorType.kBrushless); 

    private final CANEncoder encoderOne = motorOne.getEncoder();
    private final CANEncoder encoderTwo = motorTwo.getEncoder();

    public Shooter() {
        super();
    }
}
