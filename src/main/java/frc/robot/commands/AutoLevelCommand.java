package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.AutoConstants;

public class AutoLevelCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    PIDController x_pid = new PIDController(AutoConstants.AUTO_LEVEL_K_P, AutoConstants.AUTO_LEVEL_K_I,
            AutoConstants.AUTO_LEVEL_K_D);
    PIDController y_pid = new PIDController(AutoConstants.AUTO_LEVEL_K_P, AutoConstants.AUTO_LEVEL_K_I,
            AutoConstants.AUTO_LEVEL_K_D);

    public AutoLevelCommand(DrivetrainSubsystem drivetrainSubsystem) {
        addRequirements(drivetrainSubsystem);
        this.drivetrainSubsystem = drivetrainSubsystem;
        Shuffleboard.getTab("Autolevel X PID").add(x_pid);
        Shuffleboard.getTab("Autolevel Y PID").add(y_pid);
    }

    private double simAlpha = 0;

    private double deadzone(double angle) {
        if (Math.abs(angle) < Math.toRadians(5)) {
            return 0;
        }
        return angle;
    }

    private Rotation2d getCurrentRoll() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(-this.drivetrainSubsystem.getNavx().getPitch());
        }
        double dPhi_x = Math.tan(simAlpha);
        double dPhi_L = Math.sin(getCurrentYaw().getRadians()) * dPhi_x;
        return Rotation2d.fromRadians(Math.atan(dPhi_L));
    }

    private Rotation2d getCurrentPitch() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(this.drivetrainSubsystem.getNavx().getRoll());
        }
        double dPhi_x = Math.tan(simAlpha);
        double dPhi_L = Math.cos(getCurrentYaw().getRadians()) * dPhi_x;
        return Rotation2d.fromRadians(Math.atan(dPhi_L));
    }

    private Rotation2d getCurrentYaw() {
        return this.drivetrainSubsystem.getPoseMeters().getRotation();
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation()) {
            drivetrainSubsystem.resetOdometry(new Pose2d(1, 3, Rotation2d.fromDegrees(0)));
        }
    }

    // private double norm(double v) {
    // while (v > Math.PI) {
    // v -= 2 * Math.PI;
    // }
    // while (v < -Math.PI) {
    // v += 2 * Math.PI;
    // }
    // return v;
    // }

    @Override
    public void execute() {
        ChassisSpeeds correction = new ChassisSpeeds(
                x_pid.calculate(Math.tan(deadzone(getCurrentPitch().getRadians())), 0),
                -y_pid.calculate(Math.tan(deadzone(getCurrentRoll().getRadians())), 0), 0);
        SmartDashboard.putNumber("GYRO ROLL", Math.toDegrees(deadzone(getCurrentRoll().getRadians())));
        SmartDashboard.putNumber("GYRO PITCH", Math.toDegrees(deadzone(getCurrentPitch().getRadians())));
        SmartDashboard.putString("Calculated correction", "" + correction);
        if (RobotBase.isSimulation()) {
            simAlpha = (drivetrainSubsystem.getPoseMeters().getX() - 4) * 0.5;
            simAlpha = Math.min(Math.toRadians(15), simAlpha);
            simAlpha = Math.max(Math.toRadians(-15), simAlpha);
            // Simulate sliding:
            ChassisSpeeds fieldRelative = new ChassisSpeeds(4.0 * simAlpha, 0, 0);
            ChassisSpeeds rbtRel = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, getCurrentYaw());
            drivetrainSubsystem
                    .setChassisSpeeds(new ChassisSpeeds(rbtRel.vxMetersPerSecond + correction.vxMetersPerSecond,
                            rbtRel.vyMetersPerSecond + correction.vyMetersPerSecond, correction.omegaRadiansPerSecond));
        } else {
            drivetrainSubsystem.setChassisSpeeds(correction);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
