package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FinetuneFieldPose extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Pose2d pose;

    public FinetuneFieldPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d pose) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.pose = pose;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrainSubsystem.getDriveController().calculate(drivetrainSubsystem.getPoseMeters(),
                pose, 0,
                pose.getRotation());
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
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
