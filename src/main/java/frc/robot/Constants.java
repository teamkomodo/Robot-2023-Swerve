// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final double XBOX_JOYSTICK_THRESHOLD = 0.05D;

  public static final int TELESCOPE_MOTOR_ID = 31;
  public static final int TELESCOPE_ZERO_SWITCH_CHANNEL = 0;

  public static final int ELEVATOR_MOTOR_ID = 30;
  public static final int ELEVATOR_ZERO_SWITCH_CHANNEL = 1;

  public static final int CLAW_SOLENOID_FORWARD_CHANNEL = 0;
  public static final int CLAW_SOLENOID_REVERSE_CHANNEL = 1;

  public static final int JOINT_MOTOR_ID = 0;
  public static final int JOINT_ZERO_SWITCH_CHANNEL = 0;

  public static final int LED_STRIP_PWM_CHANNEL = 0;

  public static class OperatorConstants {
    public static final int driverXBoxControllerPort = 0;
    public static final int driverJoystickPort = 1;
    public static final int driverButtonsPort = 2;
  }

  //Width of robot
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47625;
  //Length of robot
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.67945;
  
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 20;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(192.7-180);
  
  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 13;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(328.1-180);
  
  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 15;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(173.5-180);
  
  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 17;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(205.0-180);

  public static final double SLOW_MODE_MODIFIER = 0.5D;

  public static final double SWERVE_STEER_P = 1.0;
  public static final double SWERVE_STEER_I = 0.0;
  public static final double SWERVE_STEER_D = 0.1;

}
