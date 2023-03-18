package frc.robot.commands.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.TrajectorySequencer;

public class ApproximateFieldPose extends CommandBase {
    private final TrajectorySequencer seq;
    private Pose2d pose;
    private Pose2d fromPose;
    private Supplier<Pose2d> supplier = null;
    private Timer timer = new Timer();

    public ApproximateFieldPose(TrajectorySequencer seq, Pose2d pose) {
        this.pose = pose;
        this.seq = seq;
        addRequirements(seq);
        addRequirements(seq.getDrivetrainSubsystem());
    }

    public ApproximateFieldPose(TrajectorySequencer seq, Supplier<Pose2d> pose) {
        this.seq = seq;
        this.supplier = pose;
        addRequirements(seq);
        addRequirements(seq.getDrivetrainSubsystem());
    }

    private boolean finished = false;
    private double multiplier = 0.0;

    @Override
    public void initialize() {
        if (this.supplier != null) {
            this.pose = this.supplier.get();
        }
        // seq.startNonRelativeTrajectory(List.of(), pose, () -> {
        // this.finished = true;
        // });
        this.fromPose = seq.getDrivetrainSubsystem().getPoseMeters();
        double distance = this.pose.getTranslation().getDistance(this.fromPose.getTranslation());
        double totalTime = distance / AutoConstants.MAX_TRAJ_SPEED_METERS_PER_SECOND;
        totalTime += 0.5; // To account for whatever
        multiplier = 1.0 / totalTime;
        timer.reset();
        timer.start();
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
        // Pose2d target = this.fromPose.interpolate(this.pose, timer.get() * multiplier);
        Translation2d targetTranslation = this.fromPose.getTranslation().interpolate(this.pose.getTranslation(), timer.get() * multiplier);
        Rotation2d targetRotation = this.fromPose.getRotation().interpolate(this.pose.getRotation(), timer.get() * multiplier);
        Pose2d target = new Pose2d(targetTranslation, targetRotation);
        SmartDashboard.putString("Target pose", "" + target);
        ChassisSpeeds speeds = this.seq.getDrivetrainSubsystem().getDriveController()
                .calculate(this.seq.getDrivetrainSubsystem().getPoseMeters(), target, 0, target.getRotation());
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                clamp(speeds.omegaRadiansPerSecond, AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND));
        this.seq.getDrivetrainSubsystem().setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;// timer.hasElapsed(1.0 / multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        seq.getDrivetrainSubsystem().stopMotion();
    }
}
