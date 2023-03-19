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
        rot = Math.abs(rot - Math.floor(rot));
        double wRotation = Math.floor(rot);
        if (rot >= 0.25 && rot <= 0.75) {
            // 180 degree offset
            targetRotation = Rotation2d.fromRotations(0.5 + wRotation);
        } else {
            targetRotation = Rotation2d.fromRotations(wRotation);
        }
    }
    @Override
    public void execute() {
        double omega = thetaController.calculate(this.drivetrainSubsystem.getGyroYaw().getRadians(), targetRotation.getRadians());
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
