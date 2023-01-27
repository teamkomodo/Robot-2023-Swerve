package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

import java.util.List;

public class SwerveControllerCommandFactory {
    public static class SwerveControllerCommandDescriptor {
        public SwerveControllerCommandImpl baseCommand;
        public Command command;
        public Trajectory trajectory;
        public Timer staticTrajectoryTimer;
        public Pose2d relativePose;
    }

    private final DrivetrainSubsystem drivetrainSubsystem;

    public SwerveControllerCommandFactory(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(AutoConstants.MAX_TRAJ_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_TRAJ_ACCEL_METERS_PER_SECOND_SQUARED)
                .setKinematics(this.drivetrainSubsystem.getDriveKinematics());
    }

    public SwerveControllerCommandDescriptor generateCommand(Trajectory traj) {
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.P_THETA_CONTROLLER, 0, 0,
                AutoConstants.THETA_PID_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Pose2d relativePose = drivetrainSubsystem.getPoseMeters();
        SwerveControllerCommandImpl controllerCommand = new SwerveControllerCommandImpl(traj,
                () -> new Pose2d(
                        drivetrainSubsystem.getPoseMeters().getTranslation()
                                .minus(relativePose.getTranslation())
                                .plus(traj.getInitialPose().getTranslation()),
                        drivetrainSubsystem.getPoseMeters().getRotation()),
                drivetrainSubsystem.getDriveKinematics(),
                new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0), thetaController,
                drivetrainSubsystem::setSwerveModuleStates, drivetrainSubsystem);
        SwerveControllerCommandDescriptor desc = new SwerveControllerCommandDescriptor();
        desc.command = controllerCommand;
        desc.baseCommand = controllerCommand;
        desc.trajectory = traj;
        desc.staticTrajectoryTimer = new Timer();
        desc.relativePose = relativePose;
        desc.command = new SequentialCommandGroup(new InstantCommand(() -> {
            desc.staticTrajectoryTimer.reset();
            desc.staticTrajectoryTimer.start();
        }), desc.command, new InstantCommand(() -> {
            drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        }));
        return desc;
    }
}
