// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final Joystick joystick = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getX()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // RamseteCommandWrapper ramseteCommand = new RamseteCommandWrapper(
    //     this.drivetrain, 
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(new Translation2d(5.5, -5.5), new Translation2d(11, 0), new Translation2d(11-1.37, -5.5)),
    //     new Pose2d(0, 0, new Rotation2d(0)));

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> drivetrain.tankLRVolts(0, 0));

    // Trajectory ex

    // return new RamseteCommandWrapper(drivetrain, exampleTrajectory);
    return new RamseteCommandWrapper(drivetrain, AutoPaths.funky);
  }
}
