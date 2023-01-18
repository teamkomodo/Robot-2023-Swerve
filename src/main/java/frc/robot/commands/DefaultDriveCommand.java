package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase{
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier translationXSupplier, 
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier) {

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        //Use `new ChassisSpeeds()` in place of `ChassisSpeeds.fromFieldRelativeSpeeds()` 
        //to achieve robot-oriented movement instead of field oriented movment
        drivetrainSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXSupplier.getAsDouble(),
            translationYSupplier.getAsDouble(),
            rotationSupplier.getAsDouble(),
            drivetrainSubsystem.getGyroYaw()
        ));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds());
    }
}
