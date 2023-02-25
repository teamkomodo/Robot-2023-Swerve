// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int driverXBoxControllerPort = 0;
        public static final int driverJoystickPort = 1;
        public static final int driverButtonsPort = 2;
    }

    public static class AutoConstants {
        // Trajectory following
        public static final double MAX_TRAJ_SPEED_METERS_PER_SECOND = 1;
        public static final double MAX_TRAJ_ACCEL_METERS_PER_SECOND_SQUARED = 10;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 3;
        public static final double MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED = 3;
        public static final double P_X_CONTROLLER = 1.2;
        public static final double P_Y_CONTROLLER = P_X_CONTROLLER;
        public static final double I_X_CONTROLLER = 0.25;
        public static final double I_Y_CONTROLLER = I_X_CONTROLLER;
        public static final double P_THETA_CONTROLLER = 3;
        public static final double I_THETA_CONTROLLER = 0.7;
        public static final TrapezoidProfile.Constraints THETA_PID_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED);
        public static final boolean ENABLE_RANDOM_GENERATION_TWEAKAGE = true;
        public static final double MAX_RANDOM_LINEAR_TWEAKAGE_METERS = 0.03;
        public static final double MAX_RANDOM_ANGULAR_TWEAKAGE_RADIANS = 0.0;
        // Autonomous collision avoidance
        public static final double MINIMUM_ALLOWABLE_TARGET_INTERSEPT_SEPARATION = 2.5;
        public static final double TARGET_INTERCEPT_DIFFERENTIAL_SECONDS = 0.10;
        public static final double TARGET_INTERCEPT_GRAD_RATE = 0.1;
        public static final long TARGET_INTERCEPT_CHECK_PERIOD_MS = 200;
        public static final boolean ENABLE_TARGET_INTERCEPT_CHECK = false;
        // Field position lineup tolerances
        public static final double MAX_POSITIONING_ERROR_METERS = 0.06;
        public static final double MAX_ANGULAR_ERROR_RADIANS = Math.toRadians(5);
        // Game piece detector
        public static final double MAX_PIECE_OFFSET_RATIO = 0.6; // 1 would be "just within the hitbox", 0 would be
                                                                 // impossibly precise.
        public static final double PIECE_LINEUP_MIN_ALIGNMENT_TIME = 0.5; // Seconds
        public static final double P_PIECE_LINEUP = 3.0;
        public static final double I_PIECE_LINEUP = 2.0;
        public static final double D_PIECE_LINEUP = 0.2;
        // Auto leveling system
        public static final double AUTO_LEVEL_K_P = 1.2;
        public static final double AUTO_LEVEL_K_I = 0.3;
        public static final double AUTO_LEVEL_K_D = 0.2;
    }

    public static class VisionConstants {
        public static final Transform3d rbt2cam = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));
        // 0 for no filtering, 1 for an integration over all of time. Don't do 1.
        public static final double ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT = 0.9;
    }

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969;
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(294.1); // FIXME Measure and set front
    // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(356.48); // FIXME Measure and set front
    // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(327.3); // FIXME Measure and set back
    // left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(168.65); // FIXME Measure and set back

    public static final double SLOW_MODE_MODIFIER = 0.1D;
    // right steer offset
}
