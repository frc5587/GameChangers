// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.Shoot;
import frc.robot.commands.SimpleShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PowercellDetector;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveToPowercell;
import frc.robot.commands.RamseteCommandWrapper;

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

    private final DeadbandJoystick joystick = new DeadbandJoystick(0);
    private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

    private final Shoot shoot = new Shoot(shooter, limelight);
    private final SimpleShoot simpleShoot = new SimpleShoot(shooter, () -> xboxController.getY(Hand.kRight));
    private final MoveToPowercell moveToPowercell = new MoveToPowercell(powercellDetector, drivetrain);

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
        drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getX()));
        shooter.setDefaultCommand(simpleShoot);
        // Trigger rightJoy = new Trigger(() -> xboxController.getY(Hand.kRight) != 0);
        JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        JoystickButton joystickTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);

        aButton.whileActiveContinuous(shoot);
        joystickTrigger.whileActiveContinuous(moveToPowercell);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        RamseteCommandWrapper ramseteCommand = new RamseteCommandWrapper(
            this.drivetrain, 
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(5.5, -5.5), new Translation2d(11, 0), new Translation2d(11-1.37, -5.5)),
            new Pose2d(0, 0, new Rotation2d(0)));
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrain.tankLRVolts(0, 0));
    }
}
