// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.fasterxml.jackson.databind.module.SimpleModule;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

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
    // create voltage constraint

    // TODO: check to see if this is relevant
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
            DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        DrivetrainConstants.DRIVETRAIN_KINEMATICS, 10);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED).setKinematics(DrivetrainConstants.DRIVETRAIN_KINEMATICS)
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior wayposints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // List.of(new Translation2d(10.211, 0),
        // new Translation2d(10.211, -10.97)),
        // List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        // Pass config
        // new Pose2d(0, -10.97, new Rotation2d(0)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drivetrain::getPose,
        new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
            DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        DrivetrainConstants.DRIVETRAIN_KINEMATICS, drivetrain::getWheelSpeeds,
        new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
        new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankLRVolts, drivetrain);

    // // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankLRVolts(0, 0));

    // Trajectory ex

    // return new RamseteCommandWrapper(drivetrain, exampleTrajectory);
    // return new RamseteCommandWrapper(drivetrain, AutoPaths.test);
  }
}
