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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
                    PoseStrategy.MULTI_TAG_PNP, camera, VisionConstants.rbt2cam);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            throw new RuntimeException(e.toString());
        }
        Shuffleboard.getTab("Vision").addString("Transform data", () -> lastString);
    }

    private String lastString = "Not set yet";

    private Runnable onVisionData = null;

    public Runnable getOnVisionData(){
        return onVisionData;
    }

    public void setOnVisionData(Runnable r) {
        onVisionData = r;
    }

    public void resetAccumulators() {
        accumulator_rotation = 0.0;
        accumulator_translation = new Translation2d();
        accumulator_weight = 0.0;
    }

    private double accumulator_rotation = 0.0;
    private Translation2d accumulator_translation = new Translation2d();
    private double accumulator_weight = 0.0;
    private double discontinuity = 0.0;

    private double moveDiscontinuity(double in, double discontinuity) {
        while (in < discontinuity) {
            in += 2 * Math.PI;
        }
        while (in >= discontinuity + (2 * Math.PI)) {
            in -= 2 * Math.PI;
        }
        return in;
    }

    @Override
    public void periodic() {
        try {
            Optional<EstimatedRobotPose> est_pose_opt = poseEstimator.update();
            if (est_pose_opt.isPresent()) {
                Pose3d pose = est_pose_opt.get().estimatedPose;
                if (pose != null) {
                    Pose2d pose2d = pose.toPose2d();
                    accumulator_translation = accumulator_translation.plus(pose2d.getTranslation()).times(VisionConstants.ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT);
                    if (accumulator_weight <= 0.0) {
                        discontinuity = pose2d.getRotation().getRadians() + Math.PI;
                    }
                    double rotation = moveDiscontinuity(pose2d.getRotation().getRadians(), discontinuity);
                    accumulator_rotation = (accumulator_rotation + rotation) * VisionConstants.ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT;
                    accumulator_weight = (accumulator_weight + 1) * VisionConstants.ITERATIVE_LEAKY_INTEGRATION_COEFFICIENT;
                    rotation = moveDiscontinuity(accumulator_rotation / accumulator_weight, 0.0);
                    pose2d = new Pose2d(accumulator_translation.div(accumulator_weight), Rotation2d.fromRadians(rotation));
                    if (doOdometryUpdate) {
                        drive.resetOdometry(pose2d);
                        if (onVisionData != null) {
                            onVisionData.run();
                        }
                        lastString = "" + pose;
                    }
                }
            }
        } catch (Exception ignored) {

        }
    }
}