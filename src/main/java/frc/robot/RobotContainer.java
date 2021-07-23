// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Shoot;
import frc.robot.commands.SimpleShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PowercellDetector;
import frc.robot.subsystems.Shooter;
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
    private final Climber climber = new Climber();

    private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
    private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

    private final LimelightCentering limelightCentering = new LimelightCentering(drivetrain, limelight);
    private final Shoot shoot = new Shoot(shooter, limelight, conveyor, intake, limelightCentering, drivetrain);
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
        Trigger rightTrigger = new Trigger(() -> xboxController.getTrigger(Hand.kRight));

        JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
        JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
        JoystickButton rightBumper = new JoystickButton(xboxController, XboxController.Button.kBumperRight.value);

        POVButton upDPad = new POVButton(xboxController, 0);
        POVButton downDPad = new POVButton(xboxController, 180);

        // joystickThumb.whileActiveContinuous(moveToPowercell);
        // joystickTrigger.whileActiveContinuous(moveToAllPowercells);

        // shooting
        aButton.whileActiveContinuous(shoot);
        // aButton.whileActiveContinuous(() -> {shooter.setThrottle(1);}, shooter);
        rightTrigger.whileActiveContinuous(limelightCentering);

        // intake pistons
        rightBumper.and(leftTrigger.negate()).whenActive(intakePistons::extend, intakePistons);
        rightBumper.and(leftTrigger).whenActive(intakePistons::retract, intakePistons);

        // moving intake
        bButton.and(leftTrigger.negate()).whileActiveContinuous(intakeForward);
        bButton.and(leftTrigger).whileActiveContinuous(intakeBackward);

        // climb
        upDPad.whenActive(() -> climber.moveUp(), climber).whenInactive(() -> climber.stopClimber(), climber);
        downDPad.whenActive(() -> climber.moveDown(), climber).whenInactive(() -> climber.stopClimber(), climber);
    }

    public Command getAutonomousCommand() {
        //* AUTO PATHS
        return new RamseteCommandWrapper(drivetrain, AutoPaths.straight_hopefully).andThen(() -> drivetrain.tankLRVolts(0, 0));
        
        //* GALACTIC SEARCH
        // return moveToAllPowercells.endAt(new Pose2d(9, 2, new Rotation2d(0)));
    }
}
