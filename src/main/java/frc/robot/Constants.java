// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.pid.JRAD;

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
    public static class ShooterConstants {
        public static final int MOTOR_ONE = 30;
        public static final int MOTOR_TWO = 31;

        public static final JRAD SHOOTER_JRAD = new JRAD(0.001, 0, 1);

        public static final double WHEEL_RADIUS = Units.inchesToMeters(3);
    }

    public static class LimelightConstants {
        public static final double LIMELIGHT_HEIGHT = 1;                          // TODO: make correct
        public static final double LIMELIGHT_ANGLE = Units.degreesToRadians(30);  // TODO: make correct

        public static final double GOAL_HEIGHT_METERS = Units.inchesToMeters(97.5);
        public static final double INNER_OUTER_GOAL_DISTANCE_METERS = Units.inchesToMeters(29.25);
    
        public static final double G_METERS_PER_SECOND_SQUARED = 9.81;
    }
}