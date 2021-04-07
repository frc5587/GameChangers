// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Shoot;
import frc.robot.commands.SimpleShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PowercellDetector;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.IntakeBackward;
import frc.robot.commands.MoveToPowercell;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.LimelightCentering;
import frc.robot.commands.MoveToAllPowercells;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePistons;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Shooter shooter = new Shooter();
    private final Limelight limelight = new Limelight();
    private final PowercellDetector powercellDetector = new PowercellDetector();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final IntakePistons intakePistons = new IntakePistons();
    private final Conveyor conveyor = new Conveyor();

    private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
    private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

    private final LimelightCentering limelightCentering = new LimelightCentering(drivetrain, limelight);
    private final Shoot shoot = new Shoot(shooter, limelight, conveyor, intake, limelightCentering);
    private final SimpleShoot simpleShoot = new SimpleShoot(shooter, () -> xboxController.getY(Hand.kRight));
    private final IntakeForward intakeForward = new IntakeForward(intake, intakePistons, conveyor);
    private final IntakeBackward intakeBackward = new IntakeBackward(intake, conveyor);
    private final MoveToPowercell moveToPowercell = new MoveToPowercell(powercellDetector, drivetrain, intakeForward);
    private final MoveToAllPowercells moveToAllPowercells = new MoveToAllPowercells(powercellDetector, drivetrain,
            intakeForward);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
        drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getXCurveDampened()));
        shooter.setDefaultCommand(simpleShoot);

        JoystickButton joystickTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);
        JoystickButton joystickThumb = new JoystickButton(joystick, Joystick.ButtonType.kTop.value);

        Trigger leftTrigger = new Trigger(() -> xboxController.getTrigger(Hand.kLeft));
        // Trigger rightTrigger = new Trigger(() ->
        // xboxController.getTrigger(Hand.kRight));

        JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
        JoystickButton rightBumper = new JoystickButton(xboxController, XboxController.Button.kBumperRight.value);
        // JoystickButton xButton = new JoystickButton(xboxController,
        // XboxController.Button.kX.value);
        // JoystickButton leftBumper = new JoystickButton(xboxController,
        // XboxController.Button.kBumperLeft.value);

        joystickThumb.whileActiveContinuous(moveToPowercell);
        joystickTrigger.whileActiveContinuous(moveToAllPowercells);

        aButton.whileActiveContinuous(shoot);

        rightBumper.and(leftTrigger.negate()).whenActive(intakePistons::extend, intakePistons);
        rightBumper.and(leftTrigger).whenActive(intakePistons::retract, intakePistons);

        bButton.and(leftTrigger.negate()).whileActiveContinuous(intakeForward);
        bButton.and(leftTrigger).whileActiveContinuous(intakeBackward);
    }

    public Command getAutonomousCommand() {
        // return new RamseteCommandWrapper(drivetrain, AutoPaths.slolom).andThen(() ->
        // drivetrain.tankLRVolts(0, 0));
        // return new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new
        // Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // new Pose2d(3, 0, new Rotation2d(0))).andThen(() -> drivetrain.tankLRVolts(0,
        // 0));
        // return new RamseteCommandWrapper(drivetrain, new Pose2d(0, 0, new
        // Rotation2d(0)), List.of(), new Pose2d(3, 0, new Rotation2d(0))).andThen(() ->
        // drivetrain.tankLRVolts(0, 0));
        // return moveToAllPowercells.endAt(new Pose2d(9, 2, new Rotation2d(0))); //
        // galactic search

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
                        DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
                DrivetrainConstants.DRIVETRAIN_KINEMATICS, 10);
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
                AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
                        .setKinematics(DrivetrainConstants.DRIVETRAIN_KINEMATICS).addConstraint(autoVoltageConstraint);
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior wayposints, making an 's' curve path
                List.of(new Translation2d(5.5, -5.5), new Translation2d(11, 0), new Translation2d(11 - 1.37, -5.5)),
                // List.of(new Translation2d(10.211, 0),
                // new Translation2d(10.211, -10.97)),
                // List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                // Pass config
                // new Pose2d(0, -10.97, new Rotation2d(0)),
                new Pose2d(0, 0, new Rotation2d(0)), config);

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
    }
}
