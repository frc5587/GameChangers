// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

    private final CANSparkMax conveyorMotorFront = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_FRONT, MotorType.kBrushless);
    private final CANSparkMax conveyorMotorBack = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_BACK, MotorType.kBrushless);
    private final CANSparkMax conveyorMotorBackTwo = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_BACK_TWO, MotorType.kBrushless);

    /** Creates a new Conveyor. */
    public Conveyor() {
        super();

        configureSparkMax();
    }

    public void configureSparkMax() {
        
        conveyorMotorFront.restoreFactoryDefaults();
        conveyorMotorFront.setIdleMode(IdleMode.kBrake);
        conveyorMotorFront.setInverted(true);
        
        conveyorMotorBack.restoreFactoryDefaults();
        conveyorMotorBack.setIdleMode(IdleMode.kBrake);
        conveyorMotorBack.setInverted(true);

        conveyorMotorBackTwo.restoreFactoryDefaults();
        conveyorMotorBackTwo.setIdleMode(IdleMode.kBrake);
        conveyorMotorBackTwo.setInverted(false);

    }

    public void backConveyor() {
        conveyorMotorBack.set(ConveyorConstants.CONVEYOR_THROTTLE_BACK);
        conveyorMotorBackTwo.set(ConveyorConstants.CONVEYOR_THROTTLE_BACK);
    }

    public void shooterConveyor() {
        conveyorMotorFront.set(ConveyorConstants.CONVEYOR_THROTTLE_FRONT);
        conveyorMotorBack.set(ConveyorConstants.CONVEYOR_THROTTLE_BACK);
        conveyorMotorBackTwo.set(ConveyorConstants.CONVEYOR_THROTTLE_BACK);
    }

    public void intakeConveyor() {
        conveyorMotorFront.set(ConveyorConstants.CONVEYOR_THROTTLE_FRONT);
    }

    public void reverse() {
        conveyorMotorFront.set(-ConveyorConstants.CONVEYOR_THROTTLE_FRONT);
    }

    public void stopIntakeConveyor() {
        conveyorMotorFront.set(0);
    }

    public void stopShooterConveyor() {
        conveyorMotorFront.set(0);
        conveyorMotorBack.set(0);
        conveyorMotorBackTwo.set(0);
    }

    @Override
    public void periodic() {
        // System.out.println(conveyorMotorBack.isI);
    }
}
