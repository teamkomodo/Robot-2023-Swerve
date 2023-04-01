package frc.robot.auto.commands;

import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CommandTemplate extends AutoCommand {
    @SuppressWarnings("unused")
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
