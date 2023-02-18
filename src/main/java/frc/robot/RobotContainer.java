// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoLevelCommand;
import frc.robot.commands.FollowApriltagCommand;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TrajectorySequencer;
import frc.robot.subsystems.VisionPositioningSubsystem;
import frc.robot.util.VisionPipelineConnector;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

import java.util.List;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    Field2d field2d = new Field2d();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final SwerveControllerCommandFactory sccf = new SwerveControllerCommandFactory(drivetrainSubsystem);
    private final TrajectorySequencer trajectorySequencer = new TrajectorySequencer(drivetrainSubsystem, sccf, null,
            null);
    private final VisionPositioningSubsystem vision = new VisionPositioningSubsystem(drivetrainSubsystem);
    private final VisionPipelineConnector visionPipeline = new VisionPipelineConnector("VisionPipeline");

    private final CommandXboxController driverXBoxController = new CommandXboxController(
            OperatorConstants.driverXBoxControllerPort);
    private final GenericHID driverJoystick = new GenericHID(OperatorConstants.driverJoystickPort);
    private final GenericHID driverButtons = new GenericHID(OperatorConstants.driverButtonsPort);
    private boolean xBoxDrive = false;

    public RobotContainer() {
        SmartDashboard.putData("Field", field2d);
        configureBindings();
    }

    private void configureBindings() {
        Trigger slowModeButton = driverXBoxController.leftBumper();

        drivetrainSubsystem.setDefaultCommand(
                Commands.run(
                        () -> drivetrainSubsystem.drive(
                                -driverXBoxController.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                -driverXBoxController.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                -driverXBoxController.getRightX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                true),
                        drivetrainSubsystem));

        slowModeButton.whileTrue(
                Commands.run(
                        () -> drivetrainSubsystem.drive(
                                -driverXBoxController.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                                        * SLOW_MODE_MODIFIER,
                                -driverXBoxController.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                                        * SLOW_MODE_MODIFIER,
                                -driverXBoxController.getRightX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                                        * SLOW_MODE_MODIFIER,
                                true),
                        drivetrainSubsystem));
    }

    private void addTestTrajectories(boolean r) {
        trajectorySequencer.startRelativeTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(4, 0, Rotation2d.fromDegrees(90)), r, null);
        trajectorySequencer.startRelativeTrajectory(new Pose2d(4, 0, Rotation2d.fromDegrees(90)),
                List.of(),
                new Pose2d(4, 4, Rotation2d.fromDegrees(180)), r, null);
        trajectorySequencer.startRelativeTrajectory(new Pose2d(4, 4, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(0, 4, Rotation2d.fromDegrees(270)), r, null);
        trajectorySequencer.startRelativeTrajectory(new Pose2d(0, 4, Rotation2d.fromDegrees(270)),
                List.of(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), r, null);
    }

    public Command getAutonomousCommand() {
        return new FollowApriltagCommand(drivetrainSubsystem, vision);
    }
}
