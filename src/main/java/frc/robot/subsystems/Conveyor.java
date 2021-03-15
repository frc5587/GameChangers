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

    private final CANSparkMax conveyorMotorBack = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_BACK, MotorType.kBrushless);
    private final CANSparkMax conveyorMotorFront = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_FRONT, MotorType.kBrushless);

    /** Creates a new Conveyor. */
    public Conveyor() {
        configureSparkMax();
    }

    public void configureSparkMax() {
        conveyorMotorBack.restoreFactoryDefaults();
        conveyorMotorBack.setIdleMode(IdleMode.kBrake);

        conveyorMotorFront.restoreFactoryDefaults();
        conveyorMotorFront.setIdleMode(IdleMode.kBrake);

    }

    public void shooterConveyor() {
        conveyorMotorFront.set(ConveyorConstants.CONVEYOR_THROTTLE_FRONT);
        conveyorMotorBack.set(ConveyorConstants.CONVEYOR_THROTTLE_BACK);
    }

    public void shooterConveyorReverse() {
        conveyorMotorFront.set(-ConveyorConstants.CONVEYOR_MOTOR_FRONT);
        conveyorMotorBack.set(-ConveyorConstants.CONVEYOR_MOTOR_BACK);
    }

    public void intakeConveyor() {
        conveyorMotorFront.set(ConveyorConstants.CONVEYOR_THROTTLE_FRONT);
    }

    public void intakeConveyorReverse() {
        conveyorMotorFront.set(ConveyorConstants.CONVEYOR_THROTTLE_BACK);
    }

    public void stopIntakeConveyor() {
        conveyorMotorFront.set(0);
    }

    public void stopShooterConveyor() {
        conveyorMotorFront.set(0);
        conveyorMotorBack.set(0);
    }
}
