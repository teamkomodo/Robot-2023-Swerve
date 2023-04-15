package frc.robot.auto.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.auto.util.AutoCommand;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowSpecifiedPath {
    public static AutoCommand generatePathCommand(DrivetrainSubsystem drive, SwerveControllerCommandFactory sccf,
            boolean relativeToInitialTranslation, String pathName) {
        String path = "paths/" + pathName + ".wpilib.json";
        Trajectory traj = null;
        try {
            Path p = Filesystem.getDeployDirectory().toPath().resolve(path);
            traj = TrajectoryUtil.fromPathweaverJson(p);
        } catch (IOException e) {
            System.err.println("Cannot open json path " + path);
            DriverStation.reportError("Cannot open json path " + path, false);
        }
        if (traj == null) {
            return new CommandFailureTest();
        }
        AutoCommand command = AutoCommand.wrap(sccf.generateCommand(traj, relativeToInitialTranslation, null).command);
        // if (!relativeToInitialTranslation) {
        //     command = new SequentialAutoCommandGroup(new FinetuneFieldPose(drive, traj.getInitialPose(), 0.10), command);
        // }
        // Pose2d finalPose = traj.getStates().get(traj.getStates().size() - 1).poseMeters;
        return command;
    }
}