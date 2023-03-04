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
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final double XBOX_JOYSTICK_THRESHOLD = 0.05D;

    public static final int JOYSTICK_PORT = 1;// HF Joystick
    public static final int BUTTONS_PORT = 2; // A-PAC Player 1
    public static final int SELECTOR_PORT = 3; // A-PAC Player 2

    public static final int TELESCOPE_MOTOR_ID = 31;
    public static final int TELESCOPE_ZERO_SWITCH_CHANNEL = 0;
    
    public static final int ELEVATOR_MOTOR_ID = 30;
    public static final int ELEVATOR_ZERO_SWITCH_CHANNEL = 1;
    public static final double ELEVATOR_SLOW_MODE_MULTIPLIER = 0.5;

    public static final int CLAW_SOLENOID_FORWARD_CHANNEL = 0;
    public static final int CLAW_SOLENOID_REVERSE_CHANNEL = 1;

    public static final int JOINT_MOTOR_ID = 0;
    public static final int JOINT_ZERO_SWITCH_CHANNEL = 2;
    
    public static final int LED_STRIP_PWM_CHANNEL = 0;

    public static final int TOF_SENSOR_ID = 33;
    
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
        public static final double P_THETA_CONTROLLER = 5;
        public static final double I_THETA_CONTROLLER = 1.5;
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
        // Reflective tape alignment
        public static final double MIN_REFLECTIVE_OFFSET_DEGREES = 2.0;
        public static final double REFLECTIVE_MIN_ALIGN_TIME = 0.5;
        public static final double P_REFL_LINEUP = 2.0;
        public static final double I_REFL_LINEUP = 2.0;
        public static final double D_REFL_LINEUP = 0.2;
        // ToF constants
        public static final double TOF_LEAKY_COEFFICIENT = 0.85;
        public static final double TOF_HALF_SWEEP_ANGLE = Math.toRadians(12);
        public static final double TOF_ANGULAR_VELOCITY = 1.2;
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

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47625; // Width of robot in meters
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.67945; // Length of robot in meters
    public static final double DRIVETRAIN_SLOW_MODE_MODIFIER = 0.5D;
    public static final boolean FIELD_RELATIVE_DRIVE = true;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 20;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(192.7 - 180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(328.1 - 180);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 15;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(173.5 - 180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 17;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(205.0 - 180);


    public static final double SWERVE_STEER_P = 1.0;
    public static final double SWERVE_STEER_I = 1.0e-3;
    public static final double SWERVE_STEER_D = 0.1;

    public static class OperatorConstants {
        public static final int driverXBoxControllerPort = 1;
        public static final int driverJoystickPort = 0;
        public static final int driverButtonsPort = 2;
    }

    public static final double TOF_OFFSET_MM = -13; // Distance = read() + TOF_OFFSET_MM
}
