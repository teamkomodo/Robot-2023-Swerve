// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PlaygroundSubsystem;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private PlaygroundSubsystem playgroundSubsystem = new PlaygroundSubsystem();

    private CommandXboxController xboxController = new CommandXboxController(XBOX_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        Trigger leftTrigger = xboxController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, XBOX_TRIGGER_THRESHOLD);
        Trigger rightTrigger = xboxController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, XBOX_TRIGGER_THRESHOLD);

        //Trigger leftJoystickXPositive = xboxController.axisGreaterThan(XboxController.Axis.kLeftX.value, XBOX_TRIGGER_THRESHOLD);
        //Trigger leftJoystickXNegative = xboxController.axisLessThan(XboxController.Axis.kLeftX.value, -XBOX_TRIGGER_THRESHOLD);
        //Trigger leftJoystickX = leftJoystickXPositive.or(leftJoystickXNegative);

        //motor 0 - left trigger for positive speed, right for negative speed
        leftTrigger.whileTrue(Commands.run(() -> playgroundSubsystem.setMotor0Speed(xboxController.getLeftTriggerAxis()), playgroundSubsystem));
        leftTrigger.onFalse(Commands.run(() -> playgroundSubsystem.setMotor0Speed(0), playgroundSubsystem));
        rightTrigger.whileTrue(Commands.run(() -> playgroundSubsystem.setMotor0Speed(-xboxController.getLeftTriggerAxis()), playgroundSubsystem));
        rightTrigger.onFalse(Commands.run(() -> playgroundSubsystem.setMotor0Speed(0), playgroundSubsystem));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
