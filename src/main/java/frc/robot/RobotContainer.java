// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TrajectorySequencer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

import java.util.List;

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
    // The robot's subsystems and commands are defined here...
    Field2d field2d = new Field2d();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final SwerveControllerCommandFactory sccf = new SwerveControllerCommandFactory(drivetrainSubsystem);
    private final TrajectorySequencer trajectorySequencer = new TrajectorySequencer(drivetrainSubsystem, sccf, null,
            null);

    private final CommandXboxController driverXBoxController = new CommandXboxController(
            OperatorConstants.driverXBoxControllerPort);
    private final GenericHID driverJoystick = new GenericHID(OperatorConstants.driverJoystickPort);
    private final GenericHID driverButtons = new GenericHID(OperatorConstants.driverButtonsPort);
    private boolean xBoxDrive = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Field", field2d);
        }
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
        Trigger slowModeButton = driverXBoxController.leftBumper();

        drivetrainSubsystem.setDefaultCommand(
                Commands.run(
                        () -> drivetrainSubsystem.drive(
                                driverXBoxController.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                driverXBoxController.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                driverXBoxController.getRightX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                true),
                        drivetrainSubsystem));

        slowModeButton.whileTrue(
                Commands.run(
                        () -> drivetrainSubsystem.drive(
                                driverXBoxController.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                                        * SLOW_MODE_MODIFIER,
                                driverXBoxController.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                                        * SLOW_MODE_MODIFIER,
                                driverXBoxController.getRightX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                                        * SLOW_MODE_MODIFIER,
                                true),
                        drivetrainSubsystem));
    }
    private void addTestTrajectories() {
        trajectorySequencer.startRelativeTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        List.of(),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        // trajectorySequencer.startRelativeTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        //                 List.of(),
        //                 new Pose2d(-0.5, 0, Rotation2d.fromDegrees(0)));
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand(
                () -> addTestTrajectories());
    }
}
