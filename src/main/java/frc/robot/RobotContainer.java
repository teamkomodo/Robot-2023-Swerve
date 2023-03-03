// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.commands.auto.AutoDefinitions;
import frc.robot.commands.auto.AutoLevelCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TrajectorySequencer;
import frc.robot.subsystems.VisionPositioningSubsystem;
import frc.robot.util.VisionPipelineConnector;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

import static frc.robot.util.Util.*;

public class RobotContainer {
    private final Field2d field2d = new Field2d();

    // Subsystem definitions should be public for auto reasons
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
    public final JointSubsystem jointSubsystem = new JointSubsystem();
    public final ClawSubsystem clawSubsystem = new ClawSubsystem();
    //public final LEDStripSubsystem ledStripSubsystem = new LEDStripSubsystem();
    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    public final SwerveControllerCommandFactory sccf = new SwerveControllerCommandFactory(drivetrainSubsystem);
    public final TrajectorySequencer trajectorySequencer = new TrajectorySequencer(drivetrainSubsystem, sccf, null,
            null);
    public final VisionPositioningSubsystem vision = new VisionPositioningSubsystem(drivetrainSubsystem);
    public final VisionPipelineConnector detector = new VisionPipelineConnector("VisionPipeline");

    private final AutoDefinitions autonomousController = new AutoDefinitions(this);

    private final CommandXboxController driverXBoxController = new CommandXboxController(
            XBOX_CONTROLLER_PORT);
    private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
    private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
    private final GenericHID selector = new GenericHID(SELECTOR_PORT);

    public RobotContainer() {
        SmartDashboard.putData("Field", field2d);
        autonomousController.initAutonomous();
        configureBindings();
    }

    private void configureBindings() {

        Trigger aButton = driverXBoxController.a();
        Trigger bButton = driverXBoxController.b();
        Trigger xButton = driverXBoxController.x();
        Trigger yButton = driverXBoxController.y();
        Trigger leftBumper = driverXBoxController.leftBumper();
        Trigger rightBumper = driverXBoxController.rightBumper();

        Trigger leftJoystickY = driverXBoxController.axisGreaterThan(XboxController.Axis.kLeftY.value, XBOX_JOYSTICK_THRESHOLD)
        .or(driverXBoxController.axisLessThan(XboxController.Axis.kLeftY.value, -XBOX_JOYSTICK_THRESHOLD));

        Trigger rightJoystickY = driverXBoxController.axisGreaterThan(XboxController.Axis.kRightY.value, XBOX_JOYSTICK_THRESHOLD)
        .or(driverXBoxController.axisLessThan(XboxController.Axis.kRightY.value, -XBOX_JOYSTICK_THRESHOLD));

        Trigger leftTrigger = driverXBoxController.leftTrigger();
        Trigger rightTrigger = driverXBoxController.rightTrigger();

        Trigger rightDriverJoystickButton = new JoystickButton(driverJoystick, 1);
        Trigger leftDriverJoystickButton = new JoystickButton(driverJoystick, 2);

        Trigger toggleSwitch1 = new JoystickButton(driverButtons, 1);
        Trigger toggleSwitch2 = new JoystickButton(driverButtons, 2);
        Trigger toggleSwitch3 = new JoystickButton(driverButtons, 3);

        Trigger whiteButton = new JoystickButton(driverButtons, 4);
        Trigger yellowBUtton = new JoystickButton(driverButtons, 5);

        Trigger selector1 = new JoystickButton(selector, 1);
        Trigger selector2 = new JoystickButton(selector, 2);
        Trigger selector3 = new JoystickButton(selector, 3);
        Trigger selector4 = new JoystickButton(selector, 4);
        Trigger selector5 = new JoystickButton(selector, 5);

    //Position Commands
        selector1.onTrue(elevatorSubsystem.runLowNodeCommand());
        selector2.onTrue(elevatorSubsystem.runMidNodeCommand());
        selector3.onTrue(elevatorSubsystem.runHighNodeCommand());
        selector4.onTrue(elevatorSubsystem.runShelfCommand());

    // Elevator Commands
        leftJoystickY.whileTrue(Commands.run(
                () -> elevatorSubsystem.setMotorPercent(driverXBoxController.getLeftY()),
                elevatorSubsystem)).onFalse(Commands.run(() -> elevatorSubsystem.setMotorPercent(0), elevatorSubsystem));

        selector1.onTrue(elevatorSubsystem.runLowNodeCommand());

    // Drivetrain Commands
        // Normal Drive
        drivetrainSubsystem.setDefaultCommand(
                Commands.run(
                        () -> drivetrainSubsystem.drive(
                                joystickCurve(-driverJoystick.getRawAxis(1))
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                joystickCurve(-driverJoystick.getRawAxis(0))
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                joystickCurve(driverJoystick.getRawAxis(2))
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                true),
                        drivetrainSubsystem));
        // Slow Drive
        leftBumper.whileTrue(
            Commands.run(
                    () -> drivetrainSubsystem.drive(
                            joystickCurve(driverXBoxController.getLeftY())
                                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
                            joystickCurve(driverXBoxController.getLeftX())
                                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
                            joystickCurve(driverXBoxController.getRightX())
                                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * SLOW_MODE_MODIFIER,
                            false),
                    drivetrainSubsystem));

        rightDriverJoystickButton.whileTrue(new AutoLevelCommand(drivetrainSubsystem));
        leftDriverJoystickButton.onTrue(Commands.runOnce(() -> drivetrainSubsystem.zeroGyro()));

    // Claw Commands
        rightBumper.whileTrue(clawSubsystem.openCommand());
        leftBumper.whileTrue(clawSubsystem.closeCommand());

    // Joint Commands
        rightTrigger.whileTrue(Commands.run(
            () -> jointSubsystem.setMotorPercent(-driverXBoxController.getRightTriggerAxis()),
            jointSubsystem).andThen(() -> jointSubsystem.setMotorPercent(0), jointSubsystem));
        leftTrigger.whileTrue(Commands.run(
            () -> jointSubsystem.setMotorPercent(driverXBoxController.getLeftTriggerAxis()),
            jointSubsystem).andThen(() -> jointSubsystem.setMotorPercent(0), jointSubsystem));
    
        
    // Telescope Commands
        rightJoystickY.whileTrue(Commands.run(
            () -> telescopeSubsystem.setMotorPercent(driverXBoxController.getRightY()),
            telescopeSubsystem).andThen(() -> telescopeSubsystem.setMotorPercent(0), telescopeSubsystem));
    }

    public Command getAutonomousCommand() {
        return null;
        // autonomousController.initAutonomous();
        // return autonomousController.chooser.getSelected().generateCommand();
        // return new AlignToGamePiece(drivetrainSubsystem, detector, 0);
    }
}
