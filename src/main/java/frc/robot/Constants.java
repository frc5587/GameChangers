// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static class DrivetrainConstants {
        public static final int LEFT_LEADER = 20;
        public static final int LEFT_FOLLOWER = 21;

        public static final int RIGHT_LEADER = 10;
        public static final int RIGHT_FOLLOWER = 11;

        public static final boolean LEFT_SIDE_INVERTED = true;
        public static final boolean RIGHT_SIDE_INVERTED = true;
        public static final boolean LEFT_ENCODER_INVERTED = true;
        public static final boolean RIGHT_ENCODER_INVERTED = true;

        public static final int SMART_CURRENT_LIMIT = 30;
        public static final int HARD_CURRENT_LIMIT = 40;

        // TODO: Figure out real values
        // Make sure that paths with Pathfinder/WPILib respect the following as well:
        // Gyro angle value should be positive when turning counterclockwise
        public static final boolean GYRO_POSITIVE_COUNTERCLOCKWISE = false;

        // TODO: Verify that assumed constants are good
        // Turn PID constants
        // public static final FPID TURN_FPID = new FPID(0, 0.03, 0, 0);
        // public static final double TURN_PID_TOLERANCE_DEG = 2.0;
        // public static final double TURN_PID_FORWARD_THROTTLE = 0.2;
        // public static final double TURN_PID_UPDATE_PERIOD_SEC = 0.010;

        // Values from characterisation
        // TODO: Find actual values
        public static final double KS_VOLTS = 0;
        public static final double KV_VOLT_SECONDS_PER_METER = 0;
        public static final double KA_VOLT_SECONDS_PER_SQUARED_METER = 0;
        public static final double TRACK_WIDTH_METERS = 0.69;

        // TODO: Change to real values
        // Basic differential drivetrain kinematics constants
        // public static final int TICKS_PER_REV = 8192;
        // public static final double WHEEL_DIAMETER_METERS = 0.2032;
        // public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
        // public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
        //         TRACK_WIDTH_METERS);

        // Ramsete constants
        public static final double RAMSETE_KP_DRIVE_VEL = 8.5;
    }
}
