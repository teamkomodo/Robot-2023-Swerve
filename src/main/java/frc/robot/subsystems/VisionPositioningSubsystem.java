package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionPositioningSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private final DrivetrainSubsystem drive;

    public boolean doOdometryUpdate = false;

    public void setCameraProcessingIdle(boolean state) {
        camera.setDriverMode(state);
    }

    public VisionPositioningSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        this.drive = drivetrainSubsystem;
        camera = new PhotonCamera("FrontCamera");
        drive.stopMotion();
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.AVERAGE_BEST_TARGETS, camera, VisionConstants.rbt2cam);
        } catch (IOException e) {
            throw new RuntimeException(e.toString());
        }
        Shuffleboard.getTab("Vision").addString("Transform data", () -> lastString);
    }

    private String lastString = "Not set yet";

    private Runnable onVisionData = null;

    public void setOnVisionData(Runnable r) {
        onVisionData = r;
    }

    // private Pose3d accumulator = new Pose3d();
    // private double accumulator_weight = 0.0;

    @Override
    public void periodic() {
        try {
            Optional<EstimatedRobotPose> est_pose_opt = poseEstimator.update();
            if (est_pose_opt.isPresent()) {
                Pose3d pose = est_pose_opt.get().estimatedPose;
                // accumulator = new Pose3d(
                // accumulator.getTranslation().times(VisionConstants.ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT)
                // .plus(pose.getTranslation()),
                // accumulator.getRotation().times(VisionConstants.ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT)
                // .plus(pose.getRotation()));
                // accumulator_weight *=
                // VisionConstants.ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT;
                // accumulator_weight += 1.0;
                // if (accumulator_weight <= 0) {
                // DriverStation.reportError("Accumulator weight is <= 0 somehow", true);
                // throw new RuntimeException("Accumulator weight is <= 0 somehow");
                // }
                // pose = new Pose3d(accumulator.getTranslation().times(1.0 /
                // accumulator_weight),
                // accumulator.getRotation().times(1.0 / accumulator_weight));
                if (pose != null) {
                    Pose2d pose2d = pose.toPose2d();
                    pose2d = new Pose2d(pose2d.getTranslation(), pose2d.getRotation().unaryMinus());
                    if (doOdometryUpdate) {
                        drive.resetOdometry(pose2d);
                        if (onVisionData != null) {
                            onVisionData.run();
                        }
                        lastString = "" + pose;
                    }
                }
            }
        }catch(Exception ignored) {

        }
    }
}
