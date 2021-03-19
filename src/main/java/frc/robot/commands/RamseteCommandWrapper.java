/**
 * THIS IS A STUB
 * 
 * This is just to get the MoveToPowercell working 
 */



package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  public RamseteCommandWrapper(Drivetrain drivetrain, double x, double y, double angle) {
    this(drivetrain, (Trajectory) null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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