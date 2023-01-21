// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  
  private final CommandXboxController driverXBoxController = new CommandXboxController(OperatorConstants.driverXBoxControllerPort);
  private final GenericHID driverJoystick = new GenericHID(OperatorConstants.driverJoystickPort);
  private final GenericHID driverButtons = new GenericHID(OperatorConstants.driverButtonsPort);
  private boolean xBoxDrive = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Trigger slowModeButton = driverXBoxController.leftBumper();

    drivetrainSubsystem.setDefaultCommand(
      Commands.run(
      () -> drivetrainSubsystem.drive(driverXBoxController.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        driverXBoxController.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        driverXBoxController.getRightX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        true),
      drivetrainSubsystem));

    slowModeButton.whileTrue(
      Commands.run(
      () -> drivetrainSubsystem.drive(driverXBoxController.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
        driverXBoxController.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
        driverXBoxController.getRightX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
        true),
      drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; //FIXME return our autonomous command
  }
}
