package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CommandTemplate extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    public CommandTemplate(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {

    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {

    }
}
