// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {

  private final CANSparkMax conveyorMotor = new CANSparkMax(Constants.ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);

  /** Creates a new Conveyor. */
  public Conveyor() {
    // conveyorMotor.setInverted(true);
    conveyorMotor.restoreFactoryDefaults();
    conveyorMotor.setIdleMode(IdleMode.kBrake);
  }

  public void moveConveyorForward() {
    conveyorMotor.set(Constants.ConveyorConstants.CONVEYOR_THROTTLE);
  }

  public void moveConveyorBackward() {
    conveyorMotor.set(-Constants.ConveyorConstants.CONVEYOR_THROTTLE);
  }

  public void stopConveyorMovement() {
    conveyorMotor.set(0);
  }

}
