// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.controllers.JRADShooterController;
import org.frc5587.lib.controllers.UnifiedShooterController;
import org.frc5587.lib.controllers.tunerHelpers.JRADTunerHelper;
import org.frc5587.lib.controllers.tunerHelpers.UnifiedShooterControllerTunerHelper;
import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.pid.UNP;
import org.frc5587.lib.pid.PID;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ConveyorConstants {
        public static final double CONVEYOR_THROTTLE_FRONT = 0.4;
        public static final double CONVEYOR_THROTTLE_BACK = .4;

        public static final int CONVEYOR_MOTOR_FRONT = 40;
        public static final int CONVEYOR_MOTOR_BACK = 50;
        public static final int CONVEYOR_MOTOR_BACK_TWO = 51;
    }

    public static class AutoConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
        public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3;

        // Reasonable values from WPILib docs (values are robot-agnostic)
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }

    public static class DrivetrainConstants {
        public static final int LEFT_LEADER = 20;
        public static final int LEFT_FOLLOWER = 21;

        public static final int RIGHT_LEADER = 10;
        public static final int RIGHT_FOLLOWER = 11;

        public static final boolean LEFT_SIDE_INVERTED = true;
        public static final boolean RIGHT_SIDE_INVERTED = true;
        public static final boolean LEFT_ENCODER_INVERTED = false;
        public static final boolean RIGHT_ENCODER_INVERTED = true;

        public static final int SMART_CURRENT_LIMIT = 30;
        public static final int HARD_CURRENT_LIMIT = 40;

        // Make sure that paths with Pathfinder/WPILib respect the following as well:
        // Gyro angle value should be positive when turning counterclockwise
        public static final boolean INVERT_GYRO_DIRECTION = true;

        // Turn PID constants
        public static final FPID TURN_FPID = new FPID(0, 0.1, 0, 0.009);
        public static final double TURN_PID_TOLERANCE_DEG = 0.5;
        public static final double TURN_PID_FORWARD_THROTTLE = 0;
        public static final double TURN_PID_UPDATE_PERIOD_SEC = 0.010;

        // Values from characterisation
        public static final double KS_VOLTS = 0.152; // used to be 0.144
        public static final double KV_VOLT_SECONDS_PER_METER = 0.193; // used to be 0.354
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.0315; // used to be 0.0249
        public static final double TRACK_WIDTH_METERS = 0.58; // measured
        public static final double RAMSETE_KP_DRIVE_VEL = 3.27; // Raw from charact. fixed: used to be 3.27

        // Basic differential drivetrain kinematics constants
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_DIAMETER_METERS = 0.2032;
        public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        // Lag compensation
        public static final int HISTORY_LIMIT = 32;
    }

    public static class ShooterConstants {
        public static final int MOTOR_ONE = 30;
        public static final int MOTOR_TWO = 31;

        // public static final UnifiedShooterControllerTunerHelper SHOOTER_CONTROLLER =
        // new UnifiedShooterControllerTunerHelper("shooter", 0.0000, 0.12235, 7.68982,
        // 44.7915, 1.02746);
        public static final UnifiedShooterController SHOOTER_CONTROLLER = new UnifiedShooterController(0.00001, 0.12235,
                7.68982, 44.7915, 1.02746);

        public static final double WHEEL_RADIUS = Units.inchesToMeters(3);
        public static final double SHOOTER_HEIGHT = 1;
        public static final double GOAL_HEIGHT = 2.495; // previously 2.41
        public static final double G = 9.806;
        public static final double SHOOTER_ANGLE = Units.degreesToRadians(55);
        public static final double GEAR_RATIO = 16 / 18;
    }

    public static class LimelightConstants {
        public static final double LIMELIGHT_ANGLE = Units.degreesToRadians(31.5);
        public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(17);
        public static final double VERTICAL_GOAL_OFFSET = Units.inchesToMeters(19);

        public static final double GOAL_HEIGHT = ShooterConstants.GOAL_HEIGHT;
        public static final double INNER_OUTER_GOAL_DISTANCE = Units.inchesToMeters(29.25);

        public static final double VFOV = 41;
        public static final double HFOV = 54;

        public static final double G = ShooterConstants.G;
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR = 60;
        public static final double THROTTLE = 1;
        public static final double MIN_THROTTLE = 0.75;
        public static final double INTAKE_RADIUS_METERS = 0.1; // TODO: make correct
        public static final double VELOCITY_MULTIPLIER = 2;
        public static final PID PID = new PID(0, 0, 0);

        public static final int[] PISTON_PORTS = { 0, 1 };
    }

    public static class ClimberConstants {
        public static final int CLIMBER_MOTOR = 59;
        public static final double CLIMBER_THROTTLE = .5;
    }
}
