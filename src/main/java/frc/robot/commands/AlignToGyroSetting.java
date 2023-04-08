package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignToGyroSetting extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final PIDController thetaController;

    public AlignToGyroSetting(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.thetaController = new PIDController(12.0, 1.7, 0.0);
        addRequirements(drivetrainSubsystem);
    }

    private double clamp(double val, double mag) {
        if (val > mag) {
            return mag;
        }
        if (val < -mag) {
            return -mag;
        }
        return val;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Rotation2d gyro = this.drivetrainSubsystem.getGyroYaw();
        double rot = gyro.getRotations();
        Rotation2d targetRotation = Rotation2d.fromRotations(Math.round(rot * 2) / 2.0);
        double omega = thetaController.calculate(this.drivetrainSubsystem.getGyroYaw().getRadians(),
                targetRotation.getRadians());
        
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, clamp(omega, 2.5));
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
