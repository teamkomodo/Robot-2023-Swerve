// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.auto.AutoLevelCommand;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.commands.auto.AutoDefinitions;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TrajectorySequencer;
import frc.robot.subsystems.VisionPositioningSubsystem;
import frc.robot.util.LimelightConnector;
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
    public final LimelightConnector limelight = new LimelightConnector("limelight");

    private final AutoDefinitions autonomousController = new AutoDefinitions(this);

    private final CommandXboxController driverXBoxController = new CommandXboxController(
            XBOX_CONTROLLER_PORT);
    private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
    private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
    private final GenericHID selector = new GenericHID(SELECTOR_PORT);

    private int selectorState = 0;

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

        Trigger backButton = driverXBoxController.back();

        Trigger leftBumper = driverXBoxController.leftBumper();
        Trigger rightBumper = driverXBoxController.rightBumper();

        Trigger leftJoystickX = driverXBoxController.axisGreaterThan(XboxController.Axis.kLeftX.value, XBOX_JOYSTICK_THRESHOLD)
        .or(driverXBoxController.axisLessThan(XboxController.Axis.kLeftX.value, -XBOX_JOYSTICK_THRESHOLD));

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
        Trigger yellowButton = new JoystickButton(driverButtons, 5);

        Trigger selector1 = new JoystickButton(selector, 1);
        Trigger selector2 = new JoystickButton(selector, 2);
        Trigger selector3 = new JoystickButton(selector, 3);
        Trigger selector4 = new JoystickButton(selector, 4);
        Trigger selector5 = new JoystickButton(selector, 5);

    //Selector Updating
        selector1.onTrue(Commands.runOnce(() -> {selectorState = 1;}));
        selector2.onTrue(Commands.runOnce(() -> {selectorState = 2;}));
        selector3.onTrue(Commands.runOnce(() -> {selectorState = 3;}));
        selector4.onTrue(Commands.runOnce(() -> {selectorState = 4;}));
        selector5.onTrue(Commands.runOnce(() -> {selectorState = 5;}));
        selector1.or(selector2).or(selector3).or(selector4).or(selector5).onFalse(Commands.runOnce(() -> {selectorState = 0;}));
        
    // Elevator Commands
        rightJoystickY.whileTrue(Commands.run(
                () -> elevatorSubsystem.setMotorPercent(-driverXBoxController.getRightY()),
                elevatorSubsystem)).onFalse(elevatorSubsystem.runHoldPositionCommand());
        //Action Button
        yellowButton.onTrue(elevatorSubsystem.runPositionCommand(selectorState));

        //Limits toggle
        toggleSwitch2.onTrue(elevatorSubsystem.runDisableLimitsCommand());
        toggleSwitch2.onFalse(elevatorSubsystem.runEnableLimitsCommand());

        //Slow mode toggle
        toggleSwitch3.onTrue(elevatorSubsystem.runEnableSlowModeCommand());
        toggleSwitch3.onFalse(elevatorSubsystem.runDisableSlowModeCommand());

    // Drivetrain Commands
        // Drive command
        drivetrainSubsystem.setDefaultCommand(
                Commands.run(
                        () -> drivetrainSubsystem.drive(
                                joystickCurve(-driverJoystick.getRawAxis(1))
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                joystickCurve(-driverJoystick.getRawAxis(0))
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                joystickCurve(driverJoystick.getRawAxis(2))
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                FIELD_RELATIVE_DRIVE),
                        drivetrainSubsystem));

        // Slow Mode
        rightDriverJoystickButton.onTrue(drivetrainSubsystem.runEnableSlowModeCommand());
        rightDriverJoystickButton.onFalse(drivetrainSubsystem.runDisableSlowModeCommand());

        // Zero Gyro (for field relative)
        leftDriverJoystickButton.onTrue(Commands.runOnce(() -> drivetrainSubsystem.zeroGyro()));

    // Claw Commands
        // XBox commands
        rightBumper.whileTrue(clawSubsystem.openCommand());
        leftBumper.whileTrue(clawSubsystem.closeCommand());

        // OC Commands
        whiteButton.onTrue(clawSubsystem.toggleCommand());
        

    // Joint Commands
        // Joint Up
        rightTrigger.whileTrue(Commands.run(
            () -> jointSubsystem.setMotorPercent(-driverXBoxController.getRightTriggerAxis()),
            jointSubsystem).andThen(() -> jointSubsystem.setMotorPercent(0), jointSubsystem));
        // Joint Down
        leftTrigger.whileTrue(Commands.run(
            () -> jointSubsystem.setMotorPercent(driverXBoxController.getLeftTriggerAxis()),
            jointSubsystem).andThen(() -> jointSubsystem.setMotorPercent(0), jointSubsystem));
    
        
    // Telescope Commands
        leftJoystickX.whileTrue(Commands.run(
            () -> telescopeSubsystem.setMotorPercent(driverXBoxController.getLeftX()),
            telescopeSubsystem));

        leftJoystickX.onFalse(Commands.runOnce(() -> telescopeSubsystem.setMotorPercent(0), telescopeSubsystem));
    }

    public Command getAutonomousCommand() {
        return null;
        // autonomousController.initAutonomous();
        // return autonomousController.chooser.getSelected().generateCommand();
        // return new AlignToGamePiece(drivetrainSubsystem, detector, 0);
        // return new AlignToReflectiveTape(drivetrainSubsystem, limelight, TapeLevel.HIGH_TAPE);
        // return new AlignToToF(drivetrainSubsystem, new TimeOfFlight(0));
    }
}
