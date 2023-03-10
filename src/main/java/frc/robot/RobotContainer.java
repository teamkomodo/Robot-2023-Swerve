// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.auto.AutoLevelCommand;
import frc.robot.commands.auto.FinetuneFieldPose;
import frc.robot.commands.auto.GetToToFDistance;
import frc.robot.commands.auto.WaitForVisionData;
import frc.robot.commands.auto.AlignToReflectiveTape.TapeLevel;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.commands.auto.AlignToReflectiveTape;
import frc.robot.commands.auto.AlignToToF;
import frc.robot.commands.auto.AutoDefinitions;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TrajectorySequencer;
import frc.robot.subsystems.VisionPositioningSubsystem;
import frc.robot.util.LimelightConnector;
import frc.robot.util.VisionPipelineConnector;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Operator Information");

    // Subsystem definitions should be public for auto reasons
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(mainTab);
    public final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem(mainTab);
    public final JointSubsystem jointSubsystem = new JointSubsystem(mainTab);
    public final ClawSubsystem clawSubsystem = new ClawSubsystem();
    //public final LEDStripSubsystem ledStripSubsystem = new LEDStripSubsystem();
    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    //public final SwerveControllerCommandFactory sccf = new SwerveControllerCommandFactory(drivetrainSubsystem);
    //public final TrajectorySequencer trajectorySequencer = new TrajectorySequencer(drivetrainSubsystem, sccf, null,
    //        null);
    public final VisionPositioningSubsystem vision = new VisionPositioningSubsystem(drivetrainSubsystem);
    public final VisionPipelineConnector detector = new VisionPipelineConnector("VisionPipeline");
    public final LimelightConnector limelight = new LimelightConnector("limelight");

    private final AutoDefinitions autonomousController = new AutoDefinitions(this);

    private final CommandXboxController driverXBoxController = new CommandXboxController(
            XBOX_CONTROLLER_PORT);
    private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
    private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
    private final GenericHID selector = new GenericHID(SELECTOR_PORT);

    public int selectorState = 0;

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
        Trigger yellowButton = new JoystickButton(driverButtons, 5);

    // Elevator Commands
        rightJoystickY.whileTrue(Commands.run(
                () -> elevatorSubsystem.setMotorPercent(-driverXBoxController.getRightY()),
                elevatorSubsystem)).onFalse(elevatorSubsystem.runHoldPositionCommand());
        //Action Button
        yellowButton.whileTrue(Commands.run(() -> elevatorSubsystem.gotoSetPosition(getSelectorState())));

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
                                -driverJoystick.getRawAxis(1)
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                -driverJoystick.getRawAxis(0)
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                driverJoystick.getRawAxis(2)
                                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                FIELD_RELATIVE_DRIVE),
                        drivetrainSubsystem));

        // Slow Mode
        //rightDriverJoystickButton.onTrue(drivetrainSubsystem.runEnableSlowModeCommand());
        //rightDriverJoystickButton.onFalse(drivetrainSubsystem.runDisableSlowModeCommand());

        // Zero Gyro (for field relative)
        leftDriverJoystickButton.onTrue(Commands.runOnce(() -> drivetrainSubsystem.zeroGyro()));
        rightDriverJoystickButton.whileTrue(new AutoLevelCommand(drivetrainSubsystem));

    // Claw Commands
    
        // XBox commands
        rightBumper.whileTrue(clawSubsystem.openCommand());
        leftBumper.whileTrue(clawSubsystem.closeCommand());

        // OC Commands
        //whiteButton.onTrue(Commands.runOnce(() -> clawSubsystem.toggle(), clawSubsystem));
        
    // Joint Commands
        
        // Joint Up
        rightTrigger.whileTrue(Commands.run(
            () -> jointSubsystem.setMotorPercent(-driverXBoxController.getRightTriggerAxis()),
            jointSubsystem));
        rightTrigger.onFalse(jointSubsystem.runHoldPositionCommand());

        // Joint Down
        leftTrigger.whileTrue(Commands.run(
            () -> jointSubsystem.setMotorPercent(driverXBoxController.getLeftTriggerAxis()),
            jointSubsystem));
        leftTrigger.onFalse(jointSubsystem.runHoldPositionCommand());

        yellowButton.onTrue(Commands.runOnce(() -> jointSubsystem.gotoSetPosition(getSelectorState())));

        //Limits
        toggleSwitch2.onTrue(jointSubsystem.runDisableLimitsCommand());
        toggleSwitch2.onFalse(jointSubsystem.runEnableLimitsCommand());

        //Slow mode toggle
        toggleSwitch3.onTrue(jointSubsystem.runEnableSlowModeCommand());
        toggleSwitch3.onFalse(jointSubsystem.runDisableSlowModeCommand());
        
    // Telescope Commands
        leftJoystickY.whileTrue(Commands.run(
            () -> telescopeSubsystem.setMotorPercent(-driverXBoxController.getLeftY()),
            telescopeSubsystem));

        leftJoystickY.onFalse(Commands.runOnce(() -> telescopeSubsystem.setMotorPercent(0), telescopeSubsystem));
        yellowButton.onTrue(Commands.runOnce(() -> telescopeSubsystem.gotoSetPosition(getSelectorState())));

        //Limits
        toggleSwitch2.onTrue(jointSubsystem.runDisableLimitsCommand());
        toggleSwitch2.onFalse(jointSubsystem.runEnableLimitsCommand());

        //Slow mode toggle
        toggleSwitch3.onTrue(telescopeSubsystem.runEnableSlowModeCommand());
        toggleSwitch3.onFalse(telescopeSubsystem.runDisableSlowModeCommand());

    //Auto Commands
        whiteButton.whileTrue(new AlignToToF(drivetrainSubsystem, clawSubsystem.getTOF()).andThen(
            new GetToToFDistance(drivetrainSubsystem, clawSubsystem.getTOF(), 0.190),
            clawSubsystem.closeCommand()));

    }

    private int getSelectorState() {
        System.out.println("Get Selector");
        if(selector.getRawButton(1)) {
            SmartDashboard.putNumber("Selector", 1);
            return 1;
        }
        if(selector.getRawButton(2))
        return 2;
        if(selector.getRawButton(3))
        return 3;
        if(selector.getRawButton(4))
        return 4;
        if(selector.getRawButton(5))
        return 5;
        return 0;
    }

    public void teleopInit() {
        telescopeSubsystem.teleopInit();
        jointSubsystem.teleopInit();
        elevatorSubsystem.teleopInit();
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new WaitForVisionData(vision),
            new CommandBase() {
                private final Timer timer = new Timer();
                @Override
                public void initialize() {
                    timer.reset();
                    timer.start();
                    limelight.setLEDs(true);
                }
                @Override
                public void execute() {}
                @Override
                public boolean isFinished() {
                    return timer.hasElapsed(0.2);
                }
                @Override
                public void end(boolean interrupted) {
                    limelight.setLEDs(false);
                }
        });
        // autonomousController.initAutonomous();
        // return autonomousController.chooser.getSelected().generateCommand();
        // return new AlignToGamePiece(drivetrainSubsystem, detector, 0);
        //return new AlignToReflectiveTape(drivetrainSubsystem, limelight, TapeLevel.HIGH_TAPE);
        // return new AlignToToF(drivetrainSubsystem, new TimeOfFlight(0));
        // return new FinetuneFieldPose(drivetrainSubsystem, new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(90)));
    }
}
