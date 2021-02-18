// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private final TalonSRX conveyorMotor = new TalonSRX(Constants.ConveyorConstants.CONVEYOR_MOTOR);

  /** Creates a new Conveyor. */
  public Conveyor() {
    conveyorMotor.setInverted(true);
  }

  public void moveConveyorForward() {
    conveyorMotor.set(ControlMode.PercentOutput, Constants.ConveyorConstants.CONVEYOR_THROTTLE);
  }

  public void moveConveyorBackward() {
    conveyorMotor.set(ControlMode.PercentOutput, -Constants.ConveyorConstants.CONVEYOR_THROTTLE);
  }

  public void stopConveyorMovement() {
    conveyorMotor.set(ControlMode.PercentOutput, 0);
  }

}
