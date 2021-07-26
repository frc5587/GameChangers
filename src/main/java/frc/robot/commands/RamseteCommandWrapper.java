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

    private boolean isReverse = false;

    private Command pathFollowCommand;
    private RamseteCommand ramsete;

    /**
     * Creates a new RamseteCommandWrapper.
     */
    public RamseteCommandWrapper(Drivetrain drivetrain, AutoPaths path) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.trajectory = path.trajectory;

        makeRamsete();
    }

    public RamseteCommandWrapper(Drivetrain drivetrain, AutoPaths path, boolean reverse) {
        addRequirements(drivetrain);

        setReverse(reverse);

        this.drivetrain = drivetrain;
        this.trajectory = path.trajectory;

        makeRamsete();
    }

    public RamseteCommandWrapper(Drivetrain drivetrain, Trajectory trajectory) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        makeRamsete();
    }

    public void setReverse(boolean reverse) {
        this.isReverse = reverse;
    }

    public RamseteCommandWrapper(Drivetrain drivetrain, Pose2d start, List<Translation2d> path, Pose2d end) {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
                        DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
                DrivetrainConstants.DRIVETRAIN_KINEMATICS, 10);

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
                AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
                        .setKinematics(DrivetrainConstants.DRIVETRAIN_KINEMATICS).addConstraint(autoVoltageConstraint);

        this.trajectory = TrajectoryGenerator.generateTrajectory(start, path, end, config);
        this.drivetrain = drivetrain;
        makeRamsete();
    }

    private void makeRamsete() {
        if (isReverse) {
            makeRamseteBackwards();
        } else {
            ramsete = new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(),
            new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS,
                    DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
                    DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            DrivetrainConstants.DRIVETRAIN_KINEMATICS, drivetrain::getWheelSpeeds,
            new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
            new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankLRVolts, drivetrain);
        }
    }

    private void makeRamseteBackwards() {
        ramsete = new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(),
        new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS,
                DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
                DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        DrivetrainConstants.DRIVETRAIN_KINEMATICS, drivetrain::getWheelSpeedsReverse,
        new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
        new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankLRVoltsReverse, drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setIdleMode(IdleMode.kBrake);
        // drivetrain.resetOdometry();

        // Start the pathFollowCommand
        if (trajectory != null) {

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

    public enum AutoPaths {
        barrel_racing("barrel_racing"), test1("test1"), circle("circle"), slolom("slolom"), bounce("bounce"),
        bounce1("bounce1"), bounce2("bounce2"), bounce3("bounce3"), bounce4("bounce4"), straight_hopefully("straight_hopefully");

        public final Path path;
        public Trajectory trajectory;
        public boolean reversed;

        private AutoPaths(String fileName, boolean reversed) {
            path = Filesystem.getDeployDirectory().toPath().resolve("paths/output/" + fileName + ".wpilib.json");
            try {
                this.trajectory = TrajectoryUtil.fromPathweaverJson(path);
            } catch (IOException e) {
                this.trajectory = null;
            }
        }

        private AutoPaths(String fileName) {
            this(fileName, false);
        }
    }
}