package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TrajectorySequencer;

public class FinetuneFieldPose extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private Pose2d pose;
    private Supplier<Pose2d> supplier = null;

    public static Command getPositioningCommand(TrajectorySequencer t, Pose2d pose) {
        return new SequentialCommandGroup(new ApproximateFieldPose(t, pose), new FinetuneFieldPose(t.getDrivetrainSubsystem(), pose));
    }

    public static Command getPositioningCommand(TrajectorySequencer t, Supplier<Pose2d> supplier) {
        return new SequentialCommandGroup(new ApproximateFieldPose(t, supplier), new FinetuneFieldPose(t.getDrivetrainSubsystem(), supplier));
    }

    public FinetuneFieldPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d pose) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.pose = pose;
        addRequirements(drivetrainSubsystem);
    }

    public FinetuneFieldPose(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> pose) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.supplier = pose;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if (supplier != null) {
            this.pose = this.supplier.get();
        }
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrainSubsystem.getDriveController().calculate(drivetrainSubsystem.getPoseMeters(),
                pose, 0,
                pose.getRotation());
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        drivetrainSubsystem.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        double d = pose.getTranslation().getDistance(drivetrainSubsystem.getPoseMeters().getTranslation());
        double a = Math.abs(pose.getRotation().minus(drivetrainSubsystem.getPoseMeters().getRotation()).getRadians());
        return (d < AutoConstants.MAX_POSITIONING_ERROR_METERS && a < AutoConstants.MAX_ANGULAR_ERROR_RADIANS);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMotion();
    }
}
