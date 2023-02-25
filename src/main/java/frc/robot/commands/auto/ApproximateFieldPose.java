package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TrajectorySequencer;

public class ApproximateFieldPose extends CommandBase {
    private final TrajectorySequencer seq;
    private final Pose2d pose;
    public ApproximateFieldPose(TrajectorySequencer seq, Pose2d pose) {
        this.pose = pose;
        this.seq = seq;
        addRequirements(seq);
    }
    private boolean finished = false;
    @Override
    public void initialize() {
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

    }
}
