// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.auto.commands.AlignToGamePiece;
import frc.robot.commands.positions.GroundCommand;
import frc.robot.commands.positions.HighNodeCommand;
import frc.robot.commands.positions.LowNodeCommand;
import frc.robot.commands.positions.MidNodeCommand;
import frc.robot.commands.positions.ShelfCommand;
import frc.robot.commands.positions.StowCommand;
import frc.robot.auto.commands.AlignToReflectiveTape;
import frc.robot.auto.commands.AlignToToF;
import frc.robot.auto.commands.AutoLevelCommand;
import frc.robot.auto.commands.GetToToFDistance;
import frc.robot.auto.commands.PickUpGamePiece;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.auto.commands.AlignToReflectiveTape.TapeLevel;
import frc.robot.auto.definitions.AutoDefinitions;
import frc.robot.auto.util.AutoMode;
import frc.robot.commands.AlignToGyroSetting;
import frc.robot.commands.IntakePieceCommand;
import frc.robot.commands.PositionCommands;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionPositioningSubsystem;
import frc.robot.util.LimelightConnector;
import frc.robot.util.VisionPipelineConnector;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

@SuppressWarnings("unused")
public class RobotContainer {
    private final Field2d field2d = new Field2d();

    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Operator Information");

    // Subsystem definitions should be public for auto reasons
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(mainTab);
    public final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem(mainTab);
    public final JointSubsystem jointSubsystem = new JointSubsystem(mainTab);
    public final ClawSubsystem clawSubsystem = new ClawSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final LEDStripSubsystem ledStripSubsystem = new LEDStripSubsystem();
    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    public final SwerveControllerCommandFactory sccf = new SwerveControllerCommandFactory(drivetrainSubsystem);
    public final VisionPositioningSubsystem vision = new VisionPositioningSubsystem(drivetrainSubsystem);
    public final VisionPipelineConnector detector = new VisionPipelineConnector("VisionPipeline");
    public final LimelightConnector limelight = new LimelightConnector("limelight");

    private final AutoDefinitions autonomousController = new AutoDefinitions(this);

    private final CommandXboxController driverXBoxController = new CommandXboxController(
            XBOX_CONTROLLER_PORT);
    private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
    private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
    private final GenericHID selector = new GenericHID(SELECTOR_PORT);

    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrainSubsystem::getPose,
        drivetrainSubsystem::resetPose,
        drivetrainSubsystem.getKinematics(),
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(1.5, 0.0, 0.0),
        drivetrainSubsystem::setModuleStates,
        eventMap, true, drivetrainSubsystem
    );

    public int selectorState = 0;

    public RobotContainer() {
        SmartDashboard.putData("Field", field2d);

        autonomousController.initAutonomous();

        configureAutoEvents();
        configureBindings();
    }

    private void configureAutoEvents() {
        eventMap.put("print", new PrintCommand("Autonomous Message"));
        eventMap.put("shelf", new ShelfCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, () -> (false)));
        eventMap.put("eject", Commands.runEnd(() -> {
            ledStripSubsystem.setPattern(-0.01); // Color 1 Larson Scanner
            intakeSubsystem.setMotorDutyCycle(1.0);
        }, () -> {
            intakeSubsystem.setMotorDutyCycle(0);
        }, intakeSubsystem, ledStripSubsystem).withTimeout(1.0));
    }

    private void configureBindings() {

        Trigger aButton = driverXBoxController.a();
        Trigger bButton = driverXBoxController.b();
        Trigger xButton = driverXBoxController.x();
        Trigger yButton = driverXBoxController.y();

        Trigger leftJoystickDown = driverXBoxController.leftStick();
        Trigger rightJoystickDown = driverXBoxController.rightStick();

        Trigger backButton = driverXBoxController.back();
        Trigger startButton = driverXBoxController.start();

        Trigger leftBumper = driverXBoxController.leftBumper();
        Trigger rightBumper = driverXBoxController.rightBumper();

        Trigger leftJoystickY = driverXBoxController
                .axisGreaterThan(XboxController.Axis.kLeftY.value, XBOX_JOYSTICK_THRESHOLD)
                .or(driverXBoxController.axisLessThan(XboxController.Axis.kLeftY.value, -XBOX_JOYSTICK_THRESHOLD));

        Trigger rightJoystickY = driverXBoxController
                .axisGreaterThan(XboxController.Axis.kRightY.value, XBOX_JOYSTICK_THRESHOLD)
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
        Trigger blueButton = new JoystickButton(driverButtons, 7);
        Trigger blueTriButton = new JoystickButton(driverButtons, 6);

        BooleanSupplier cubeMode = () -> toggleSwitch1.getAsBoolean();

        ledStripSubsystem.setDefaultCommand(Commands.run(() -> {
            if (toggleSwitch1.getAsBoolean()) {
                ledStripSubsystem.setPattern(LEDStripSubsystem.CUBE_SIGNAL_PATTERN);
            } else {
                ledStripSubsystem.setPattern(LEDStripSubsystem.CONE_SIGNAL_PATTERN);
            }
        }, ledStripSubsystem));

        toggleSwitch1.onTrue(ledStripSubsystem.cubeSignalCommand()).onFalse(ledStripSubsystem.coneSignalCommand());

        aButton.onTrue(new StowCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem));
        bButton.onTrue(new LowNodeCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, cubeMode));
        xButton.onTrue(new MidNodeCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem,  cubeMode));
        yButton.onTrue(new HighNodeCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, cubeMode));
        startButton.onTrue(new ShelfCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, cubeMode));
        rightJoystickDown.onTrue(new GroundCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, cubeMode));

        whiteButton.whileTrue(Commands.parallel(elevatorSubsystem.zeroCommand(), telescopeSubsystem.zeroCommand(), jointSubsystem.zeroCommand()));
        
        // Elevator Commands
        rightJoystickY.whileTrue(Commands.run(
                () -> elevatorSubsystem.setMotorPercent(-driverXBoxController.getRightY()),
                elevatorSubsystem)).onFalse(elevatorSubsystem.holdPositionCommand());

        // Limits toggle
        toggleSwitch2.onTrue(elevatorSubsystem.disableLimitsCommand()).onFalse(elevatorSubsystem.enableLimitsCommand());

        // Slow mode toggle
        toggleSwitch3.onTrue(elevatorSubsystem.enableSlowModeCommand())
                .onFalse(elevatorSubsystem.disableSlowModeCommand());

        // Drivetrain Commands
        // Drive command

        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> (driverJoystick.getRawAxis(1)), // +Y on joystick is +X for robot
                () -> (driverJoystick.getRawAxis(0)), // +X on joystick is +Y for robot
                () -> (-driverJoystick.getRawAxis(2))) // -Z on joystick is +Z for robot
        );

        // Slow Mode
        blueButton.onTrue(drivetrainSubsystem.disableSlowModeCommand())
                .onFalse(drivetrainSubsystem.enableSlowModeCommand());

        // Zero Gyro
        leftDriverJoystickButton.onTrue(Commands.runOnce(() -> drivetrainSubsystem.zeroGyro()));

        // Claw Commands

        // rightBumper.whileTrue(clawSubsystem.openCommand());
        // leftBumper.whileTrue(clawSubsystem.closeCommand());

        // Intake Commands

        // Intake
        rightBumper.whileTrue(new IntakePieceCommand(intakeSubsystem, jointSubsystem, ledStripSubsystem));

        // Eject
        leftBumper.whileTrue(Commands.runEnd(() -> {
            ledStripSubsystem.setPattern(-0.01); // Color 1 Larson Scanner
            intakeSubsystem.setMotorDutyCycle(1.0);
        }, () -> {
            intakeSubsystem.setMotorDutyCycle(0);
        }, intakeSubsystem, ledStripSubsystem));

        // Joint Commands

        jointSubsystem.setDefaultCommand(Commands.run(() -> {
            if (elevatorSubsystem.getPosition() > ELEVATOR_JOINT_DANGER_THRESHOLD
                    && jointSubsystem.getPosition() < JOINT_DANGER_POSITION) {
                jointSubsystem.setPosition(JOINT_DANGER_POSITION);
            }
        }, jointSubsystem));
        // Joint Up
        rightTrigger.whileTrue(Commands.run(
                () -> jointSubsystem.setMotorPercent(-driverXBoxController.getRightTriggerAxis()),
                jointSubsystem));
        rightTrigger.onFalse(jointSubsystem.holdPositionCommand());

        // Joint Down
        leftTrigger.whileTrue(Commands.run(
                () -> jointSubsystem.setMotorPercent(driverXBoxController.getLeftTriggerAxis()),
                jointSubsystem));
        leftTrigger.onFalse(jointSubsystem.holdPositionCommand());

        // Limits
        toggleSwitch2.onTrue(jointSubsystem.disableLimitsCommand()).onFalse(jointSubsystem.enableLimitsCommand());

        // Slow mode toggle
        toggleSwitch3.onTrue(jointSubsystem.enableSlowModeCommand()).onFalse(jointSubsystem.disableSlowModeCommand());

        // Telescope Commands
        leftJoystickY.whileTrue(Commands.run(
                () -> telescopeSubsystem.setMotorPercent(-driverXBoxController.getLeftY()),
                telescopeSubsystem));

        leftJoystickY.onFalse(Commands.runOnce(() -> telescopeSubsystem.setMotorPercent(0), telescopeSubsystem));

        // Limits
        toggleSwitch2.onTrue(jointSubsystem.disableLimitsCommand()).onFalse(jointSubsystem.enableLimitsCommand());

        // Slow mode toggle
        toggleSwitch3.onTrue(telescopeSubsystem.enableSlowModeCommand())
                .onFalse(telescopeSubsystem.disableSlowModeCommand());

        // Auto Commands
        backButton.and(cubeMode).whileTrue(
                new GetToToFDistance(drivetrainSubsystem, clawSubsystem.getTOF(), TOF_DISTANCE_METERS_CUBE).andThen(
                        clawSubsystem.closeCommand()));

        backButton.and(() -> !cubeMode.getAsBoolean()).whileTrue(
                new GetToToFDistance(drivetrainSubsystem, clawSubsystem.getTOF(), TOF_DISTANCE_METERS_CONE).andThen(
                        clawSubsystem.closeCommand()));

        rightDriverJoystickButton.whileTrue(new AutoLevelCommand(drivetrainSubsystem));
        blueTriButton.whileTrue(new AlignToGyroSetting(drivetrainSubsystem));

        leftJoystickDown.whileTrue(new AlignToToF(drivetrainSubsystem, clawSubsystem.getTOF()));

        yellowButton.whileTrue(new AlignToReflectiveTape(drivetrainSubsystem, limelight, TapeLevel.HIGH_TAPE));
        whiteButton.whileTrue(new SequentialCommandGroup(new AlignToGamePiece(drivetrainSubsystem, vision, detector, 0),
                new PickUpGamePiece(drivetrainSubsystem, intakeSubsystem, jointSubsystem)));
    }

    private Command autoCommand = null;

    public void teleopInit() {
        telescopeSubsystem.teleopInit();
        jointSubsystem.teleopInit();
        elevatorSubsystem.teleopInit();
        if (autoCommand != null) {
            autoCommand.end(true);
        }
        ledStripSubsystem.setPattern(LEDStripSubsystem.IDLE_PATTERN);
        drivetrainSubsystem.stopMotion();
        drivetrainSubsystem.setGyro(Rotation2d.fromDegrees(180));
    }

    public Command getAutonomousCommand() {
        // AutoMode mode = autonomousController.chooser.getSelected();
        // if (mode == null) {
        //     return null;
        // } else {
        //     autoCommand = mode.generateAutonomousEngine(this);
        //     return autoCommand;
        // }
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Demo", new PathConstraints(1.0, 2));
        return autoBuilder.fullAuto(pathGroup);
    }
}
