// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

import static frc.robot.Constants.*;
import static frc.robot.util.Util.*;

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

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    //private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
    //private final JointSubsystem jointSubsystem = new JointSubsystem();
    //private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final LEDStripSubsystem ledStripSubsystem = new LEDStripSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    
    private final CommandXboxController xboxController = new CommandXboxController(OperatorConstants.driverXBoxControllerPort);
    private final GenericHID driverJoystick = new GenericHID(OperatorConstants.driverJoystickPort);
    private final GenericHID driverButtons = new GenericHID(OperatorConstants.driverButtonsPort);
    private boolean xBoxDrive = false;
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
        Trigger aButton = xboxController.a();
        Trigger bButton = xboxController.b();
        Trigger xButton = xboxController.x();
        Trigger yButton = xboxController.y();
        Trigger rightBumper = xboxController.rightBumper();
        Trigger leftBumper = xboxController.leftBumper();

        Trigger leftJoystickYPositive = xboxController.axisGreaterThan(XboxController.Axis.kLeftY.value, XBOX_JOYSTICK_THRESHOLD);
        Trigger leftJoystickYNegative = xboxController.axisLessThan(XboxController.Axis.kLeftY.value, -XBOX_JOYSTICK_THRESHOLD);
        Trigger leftJoystickY = leftJoystickYPositive.or(leftJoystickYNegative);

        Trigger leftTrigger = xboxController.leftTrigger();
        Trigger rightTrigger = xboxController.rightTrigger();

        //Elevator Triggers
        aButton.onTrue(elevatorSubsystem.runLowNodeCommand());
        bButton.onTrue(elevatorSubsystem.runMidNodeCommand());
        yButton.onTrue(elevatorSubsystem.runHighNodeCommand());
        xButton.onTrue(elevatorSubsystem.runShelfCommand());
        leftTrigger.whileTrue(Commands.run(
            () -> elevatorSubsystem.setMotorPercent(xboxController.getLeftTriggerAxis()),
            elevatorSubsystem).andThen(() -> elevatorSubsystem.setMotorPercent(0)));
        rightTrigger.whileTrue(Commands.run(
            () -> elevatorSubsystem.setMotorPercent(xboxController.getRightTriggerAxis()),
            elevatorSubsystem).andThen(() -> elevatorSubsystem.setMotorPercent(0)));

        //Claw Triggers
        // rightBumper.whileTrue(clawSubsystem.openCommand());
        // leftBumper.whileTrue(clawSubsystem.closeCommand());

        drivetrainSubsystem.setDefaultCommand(
          Commands.run(
          () -> drivetrainSubsystem.drive(joystickCurve(xboxController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            joystickCurve(xboxController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            joystickCurve(xboxController.getRightX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            false),
          drivetrainSubsystem));

        leftBumper.whileTrue(
          Commands.run(
          () -> drivetrainSubsystem.drive(joystickCurve(xboxController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
            joystickCurve(xboxController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
            joystickCurve(xboxController.getRightX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
            false),
          drivetrainSubsystem));
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
