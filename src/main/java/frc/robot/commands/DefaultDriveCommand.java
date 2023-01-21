package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase{
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rightSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier forwardSupplier, 
    DoubleSupplier rightSupplier,
    DoubleSupplier rotationSupplier) {

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.rightSupplier = rightSupplier;
        this.rotationSupplier = rotationSupplier;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        //Use `new ChassisSpeeds()` in place of `ChassisSpeeds.fromFieldRelativeSpeeds()` 
        //to achieve robot-oriented movement instead of field oriented movment
        drivetrainSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSupplier.getAsDouble(),
            rightSupplier.getAsDouble(),
            rotationSupplier.getAsDouble(),
            drivetrainSubsystem.getGyroYaw()
        ));

        //Robot Realative Chassis Speeds
        // drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(
        //     translationXSupplier.getAsDouble(),
        //     translationYSupplier.getAsDouble(),
        //     rotationSupplier.getAsDouble()
        // ));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds());
    }
}
