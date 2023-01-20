package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

public class SwerveControllerCommandFactory {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public SwerveControllerCommandFactory(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(AutoConstants.MAX_TRAJ_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_TRAJ_ACCEL_METERS_PER_SECOND_SQUARED)
                .setKinematics(this.drivetrainSubsystem.getDriveKinematics());
    }

    public Command generateCommand(Trajectory traj) {
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.P_THETA_CONTROLLER, 0, 0,
                AutoConstants.THETA_PID_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Pose2d relativePose = drivetrainSubsystem.getPoseMeters().relativeTo(traj.getInitialPose());
        SwerveControllerCommand controllerCommand = new SwerveControllerCommand(traj,
                () -> drivetrainSubsystem.getPoseMeters().relativeTo(relativePose),
                drivetrainSubsystem.getDriveKinematics(),
                new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0), thetaController,
                drivetrainSubsystem::setSwerveModuleStates, drivetrainSubsystem);

        return controllerCommand.andThen(() -> drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
