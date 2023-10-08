package frc.robot.auto.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FinetuneFieldPose extends AutoCommand {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private Pose2d pose;
    private Supplier<Pose2d> supplier = null;
    private final double error;

    // public static Command getPositioningCommand(TrajectorySequencer t, Pose2d pose) {
    //     return new SequentialCommandGroup(new ApproximateFieldPose(t, pose),
    //             new FinetuneFieldPose(t.getDrivetrainSubsystem(), pose));
    // }

    // public static Command getPositioningCommand(TrajectorySequencer t, Supplier<Pose2d> supplier) {
    //     return new SequentialCommandGroup(new ApproximateFieldPose(t, supplier),
    //             new FinetuneFieldPose(t.getDrivetrainSubsystem(), supplier));
    // }

    public FinetuneFieldPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d pose, double error) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.pose = pose;
        addRequirements(drivetrainSubsystem);
        if (error <= 0) {
            error = AutoConstants.MAX_POSITIONING_ERROR_METERS;
        }
        this.error = error;
    }

    public FinetuneFieldPose(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> pose, double error) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.supplier = pose;
        if (error <= 0) {
            error = AutoConstants.MAX_POSITIONING_ERROR_METERS;
        }
        this.error = error;        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if (supplier != null) {
            this.pose = this.supplier.get();
        }
    }

    private double clamp(double val, double mag) {
        if (val > mag) {
            return mag;
        }
        if (val < -mag) {
            return -mag;
        }
        return val;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrainSubsystem.getDriveController().calculate(drivetrainSubsystem.getPose(),
                pose, 0,
                pose.getRotation());
        speeds = new ChassisSpeeds(clamp(-speeds.vxMetersPerSecond, AutoConstants.MAX_TRAJ_SPEED_METERS_PER_SECOND),
                clamp(-speeds.vyMetersPerSecond, AutoConstants.MAX_TRAJ_SPEED_METERS_PER_SECOND),
                clamp(speeds.omegaRadiansPerSecond, AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND));
        drivetrainSubsystem.drive(speeds, false);
    }

    @Override
    public boolean isFinished() {
        double d = pose.getTranslation().getDistance(drivetrainSubsystem.getPose().getTranslation());
        double a = Math.abs(pose.getRotation().minus(drivetrainSubsystem.getPose().getRotation()).getRadians());
        return (d < error && a < AutoConstants.MAX_ANGULAR_ERROR_RADIANS);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMotion();
    }
}
