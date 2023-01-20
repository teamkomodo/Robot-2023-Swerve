package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final SwerveControllerCommandFactory controllerCommandFactory;
    private Command swerveControllerCommand;

    public AutonomousCommand(DrivetrainSubsystem drive, SwerveControllerCommandFactory controllerCommandFactory) {
        this.drivetrainSubsystem = drive;
        this.controllerCommandFactory = controllerCommandFactory;
    }

    private void startRelativeTrajectory(Pose2d initialPose, List<Translation2d> waypoints, Pose2d finalPose,
            boolean addToCurrentCommand) {
        Command controllerCommand = controllerCommandFactory.generateCommand(TrajectoryGenerator
                .generateTrajectory(initialPose, waypoints, finalPose, controllerCommandFactory.getTrajectoryConfig()));
        if (addToCurrentCommand && swerveControllerCommand != null) {
            swerveControllerCommand = swerveControllerCommand.andThen(controllerCommand);
        } else {
            if (swerveControllerCommand != null) {
                swerveControllerCommand.end(true);
            }
            swerveControllerCommand = controllerCommand;
            swerveControllerCommand.initialize();
        }
    }

    @Override
    public void initialize() {
        startRelativeTrajectory(new Pose2d(0, 0, Rotation2d.fromRadians(0)),
                List.of(new Translation2d(4, 4)),
                new Pose2d(0, 0, Rotation2d.fromRadians(Math.PI / 2.0)), false);
    }

    @Override
    public void execute() {
        if (swerveControllerCommand != null) {
            swerveControllerCommand.execute();
            if (swerveControllerCommand.isFinished()) {
                swerveControllerCommand.end(false);
                swerveControllerCommand = null;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}
