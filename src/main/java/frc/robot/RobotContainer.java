// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Conveyor conveyor = new Conveyor();

    private final DeadbandJoystick joy = new DeadbandJoystick(0);
    private final DeadbandXboxController xb = new DeadbandXboxController(1);

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

        var leftBumper = new JoystickButton(xb, DeadbandXboxController.Button.kBumperLeft.value);
        var rightBumper = new JoystickButton(xb, DeadbandXboxController.Button.kBumperRight.value);
        var yButton = new JoystickButton(xb, DeadbandXboxController.Button.kY.value);
        var bButton = new JoystickButton(xb, DeadbandXboxController.Button.kB.value);

        // Conveyor w/o intake
        rightBumper.whileHeld(() -> {conveyor.intakeConveyor();}, conveyor)
            .whenReleased(() -> {conveyor.stopIntakeConveyor();}, conveyor);

        leftBumper.whileHeld(() -> {conveyor.reverse();;}, conveyor)
            .whenReleased(() -> {conveyor.stopIntakeConveyor();}, conveyor);

        yButton.whileHeld(() -> {conveyor.shooterConveyor();}, conveyor)
            .whenReleased(() -> {conveyor.stopShooterConveyor();}, conveyor);
        
        bButton.whileHeld(() -> {conveyor.reverse();}, conveyor)
            .whenReleased(() -> {conveyor.stopShooterConveyor();}, conveyor);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
