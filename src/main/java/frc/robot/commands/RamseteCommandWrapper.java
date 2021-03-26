/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RamseteCommandWrapper extends CommandBase {
  private final Drivetrain drivetrain;
  private final Trajectory trajectory;

  private Command pathFollowCommand;

  /**
   * Creates a new RamseteCommandWrapper.
   */
  public RamseteCommandWrapper(Drivetrain drivetrain, AutoPaths path) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;

    // Get the path to the trajectory on the RoboRIO's filesystem
    var trajectoryPath = path.getJSONPath();

    // Get the trajectory based on the file path (throws IOException if not found)
    Trajectory trajectory = null;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open " + path + " trajectory: " + trajectoryPath, ex.getStackTrace());
    }

    // trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath); // comment out try catch ^

    // Yell at us if we leave the trajectory at null
    if (trajectory == null) {
      throw new NullPointerException();
    }
    this.trajectory = trajectory;
  }

  public RamseteCommandWrapper(Drivetrain drivetrain, Trajectory trajectory) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
  }

  public RamseteCommandWrapper(Drivetrain drivetrain, Pose2d start, List<Translation2d> path, Pose2d end) {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
          DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
      DrivetrainConstants.DRIVETRAIN_KINEMATICS, 10);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
      AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED).setKinematics(DrivetrainConstants.DRIVETRAIN_KINEMATICS)
          .addConstraint(autoVoltageConstraint);

    this.trajectory = TrajectoryGenerator.generateTrajectory(start, path, end, config);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setIdleMode(IdleMode.kBrake);
    // drivetrain.resetOdometry();

    // Start the pathFollowCommand
    if (trajectory != null) {

      // Create the RamseteCommand based on the drivetrain's constants
      var ramsete = new RamseteCommand(trajectory, drivetrain::getPose,
          new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
          new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
              DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
          DrivetrainConstants.DRIVETRAIN_KINEMATICS, drivetrain::getWheelSpeeds,
          new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
          new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankLRVolts, drivetrain);
      
      drivetrain.resetOdometry(trajectory.getInitialPose());

      // Run path following command, then stop at the end
      pathFollowCommand = ramsete;

      pathFollowCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain and path following command just in case
    if (pathFollowCommand != null) {
      pathFollowCommand.cancel();
    }
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathFollowCommand != null) {
      return !pathFollowCommand.isScheduled();
    } else {
      return true;
    }
  }

  public enum AutoPaths {
    test;

    /**
     * Get the path to the corresponding path JSON file (generated with PathWeaver)
     * in the roboRIO's filesystem for a given enum value
     * 
     * @return the complete path under the roboRIO's filesystem for the
     *         corresponding path JSON
     */
    public Path getJSONPath() {
      var path = "paths/";
      switch (this) {
        case test:
          path += "test.wpilib.json";
          break;
      }

      // Join the path with where the code is deployed to on the roboRIO, in order to
      // get the complete path
      return Filesystem.getDeployDirectory().toPath().resolve(path);
    }
  }
}