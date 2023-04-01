package frc.robot.auto.commands;

import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunUntilNotLevel extends AutoCommand {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final AutoCommand command;
    public RunUntilNotLevel(DrivetrainSubsystem drivetrainSubsystem, AutoCommand command) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.command = command;
        addRequirements(drivetrainSubsystem);
    }
    private boolean endedEarly;
    @Override
    public void initialize() {
        endedEarly = false;
        this.command.initialize();
    }
    @Override
    public void execute() {
        this.command.execute();
        if (Math.cos(Math.toRadians(this.drivetrainSubsystem.getNavx().getPitch())) < 0.985) {
            endedEarly = true;
        }
        if (Math.cos(Math.toRadians(this.drivetrainSubsystem.getNavx().getRoll())) < 0.985) {
            endedEarly = true;
        }
    }
    @Override
    public boolean isFinished() {
        return this.command.isFinished() || endedEarly;
    }
    @Override
    public boolean didSucceed() {
        return this.command.didSucceed() || endedEarly;
    }
    @Override
    public void end(boolean interrupted) {
        this.command.end(interrupted || endedEarly);
    }
}
