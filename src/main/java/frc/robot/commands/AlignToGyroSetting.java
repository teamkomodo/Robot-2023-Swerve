package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignToGyroSetting extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ProfiledPIDController thetaController;
    private Rotation2d targetRotation = null;

    public AlignToGyroSetting(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.thetaController = this.drivetrainSubsystem.getDriveController().getThetaController();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Rotation2d gyro = this.drivetrainSubsystem.getGyroYaw();
        double rot = gyro.getRotations();
        targetRotation = Rotation2d.fromRotations(Math.round(rot * 2) / 2.0);
    }

    @Override
    public void execute() {
        double omega = thetaController.calculate(this.drivetrainSubsystem.getGyroYaw().getRadians(),
                targetRotation.getRadians());
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);
        drivetrainSubsystem.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMotion();
    }
}
