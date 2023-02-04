package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionPositioningSubsystem;

public class FollowApriltagCommand extends CommandBase {
    private final DrivetrainSubsystem drive;
    private final VisionPositioningSubsystem vision;
    public FollowApriltagCommand(DrivetrainSubsystem drive, VisionPositioningSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive, vision);
    }
    double lastVision = 0.0;
    @Override
    public void initialize() {
        this.vision.setOnVisionData(() -> onVisionData());
    }

    private void onVisionData() {
        ChassisSpeeds speeds = drive.getDriveController().calculate(drive.getPoseMeters(), new Pose2d(13.8, 1.0, Rotation2d.fromRadians(0)), 0,
                Rotation2d.fromRadians(0));
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        drive.setChassisSpeeds(speeds);
        lastVision = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - lastVision > 0.2) {
            drive.stopMotion();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.vision.setOnVisionData(null);
        drive.stopMotion();
    }
}
