/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer;

public class LimelightCentering extends CommandBase {
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final Notifier notifier;
    private final Timer timer = new Timer();
    private double lastAngle = 0;

    /**
     * Creates a new LimelightCentring.
     */
    public LimelightCentering(Drivetrain drivetrain, Limelight limelight) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;

        // Limelight not a requirement because it is fine if multiple commands use it
        this.limelight = limelight;

        this.notifier = new Notifier(this::updatePID);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // Run the update method based on the given period
        notifier.startPeriodic(Constants.DrivetrainConstants.TURN_PID_UPDATE_PERIOD_SEC);

        // Stop drivetrain and enable PID controller
        drivetrain.stop();
        drivetrain.enable();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain and PID controller
        drivetrain.stop();
        drivetrain.disable();

        // Stop the notifier from updating again
        notifier.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Finish once the turn PID controller is at the setpoint, indicating that it is
        // centred on the target
        return (drivetrain.atSetpoint() && timer.get() >= 0.5);
    }

    /**
     * Updates the setpoint of the drivetrain's angle PID loop based on the angle
     * from the target as determined by the Limelight and the drivetrain's current
     * angle.
     * 
     * <p>
     * Note that this method <b>does not</b> check that the angle PID is enabled or
     * that the Limelight's LEDs are currently on, even though this method will not
     * be able to do anything if these are not both set on.
     * 
     * @see Drivetrain#enable()
     */
    private void updatePID() {

        if (limelight.isTargetDetected()) {
            // Get the difference between centre and vision target (error)
            var angleError = limelight.getHorizontalAngle();

            lastAngle = Math.IEEEremainder(drivetrain.getHeading180() + angleError, 180);
        }

        double diffDeg = Math.IEEEremainder(lastAngle - drivetrain.getHeading180(), 180);
        double diffV = MathUtil.clamp(diffDeg/(LimelightConstants.HFOV/2), -1, 1) * 0.4;

        drivetrain.tankLR(diffV, -diffV);
    }
}