package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

import java.util.function.Supplier;

public class SwerveControllerCommandFactory {
    public static class SwerveControllerCommandDescriptor {
        public SwerveControllerCommandImpl baseCommand;
        public Command command;
        public Trajectory trajectory;
        public Timer staticTrajectoryTimer;
        private Pose2d relativePose;
    }

    @FunctionalInterface
    public static interface SwerveControllerCommandOnComplete {
        void onComplete();
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

    public SwerveControllerCommandDescriptor generateCommand(Trajectory traj, boolean relativeToInitialTranslation,
            Runnable onComplete) {

        SwerveControllerCommandDescriptor desc = new SwerveControllerCommandDescriptor();
        desc.relativePose = new Pose2d(0, 0, Rotation2d.fromRadians(0));

        Supplier<Pose2d> poseSupplier = () -> relativeToInitialTranslation ? new Pose2d(
                drivetrainSubsystem.getPoseMeters().getTranslation()
                        .minus(desc.relativePose.getTranslation())
                        .plus(traj.getInitialPose().getTranslation()),
                drivetrainSubsystem.getPoseMeters().getRotation()) : drivetrainSubsystem.getPoseMeters();

        SwerveControllerCommandImpl controllerCommand = new SwerveControllerCommandImpl(traj,
                poseSupplier,
                drivetrainSubsystem.getDriveKinematics(),
                drivetrainSubsystem.getDriveController(),
                drivetrainSubsystem::setSwerveModuleStates, drivetrainSubsystem);

        desc.command = controllerCommand;
        desc.baseCommand = controllerCommand;
        desc.trajectory = traj;
        desc.staticTrajectoryTimer = new Timer();

        desc.command = new SequentialCommandGroup(new InstantCommand(() -> {
            desc.staticTrajectoryTimer.reset();
            desc.staticTrajectoryTimer.start();
            if (relativeToInitialTranslation) {
                desc.relativePose = drivetrainSubsystem.getPoseMeters();
            }
        }), desc.command, new InstantCommand(() -> {
            drivetrainSubsystem.stopMotion();
        }, drivetrainSubsystem));

        if (onComplete != null) {
            ((SequentialCommandGroup) desc.command).addCommands(new InstantCommand(onComplete));
        }
        return desc;
    }
}
