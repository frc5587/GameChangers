// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.pid.FPID;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
    public static class AutoConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
        public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3;

        // Reasonable values from WPILib docs (values are robot-agnostic)
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }

    public static class DrivetrainConstants {
        public static final int LEFT_LEADER = 21;
        public static final int LEFT_FOLLOWER = 20;

        public static final int RIGHT_LEADER = 10;
        public static final int RIGHT_FOLLOWER = 11;

        public static final boolean LEFT_SIDE_INVERTED = false;
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
        public static final double KS_VOLTS = 0.181;
        public static final double KV_VOLT_SECONDS_PER_METER = 0.337;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.0179;
        public static final double TRACK_WIDTH_METERS = 0.686863135; // Empirically determined fixed
        public static final double RAMSETE_KP_DRIVE_VEL = .701; // Raw from charact. fixed

        // Basic differential drivetrain kinematics constants
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_DIAMETER_METERS = 0.2032;
        public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        // Lag compensation
        public static final int HISTORY_LIMIT = 32;
    }
}
