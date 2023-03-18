package frc.robot.commands.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TrajectorySequencer;

public class ApproximateFieldPose extends CommandBase {
    private final TrajectorySequencer seq;
    private Pose2d pose;
    private Supplier<Pose2d> supplier = null;
    public ApproximateFieldPose(TrajectorySequencer seq, Pose2d pose) {
        this.pose = pose;
        this.seq = seq;
        addRequirements(seq);
    }
    public ApproximateFieldPose(TrajectorySequencer seq, Supplier<Pose2d> pose) {
        this.seq = seq;
        this.supplier = pose;
        addRequirements(seq);
    }
    private boolean finished = false;
    @Override
    public void initialize() {
        if (this.supplier != null) {
            this.pose = this.supplier.get();
        }
        seq.startNonRelativeTrajectory(List.of(), pose, () -> {
            this.finished = true;
        });
    }
    @Override
    public void execute() {
        // Do nothing
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
    @Override
    public void end(boolean interrupted) {
        seq.getDrivetrainSubsystem().stopMotion();
    }
}
