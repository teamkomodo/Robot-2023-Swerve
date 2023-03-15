package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunUntilNotLevel extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Command command;
    public RunUntilNotLevel(DrivetrainSubsystem drivetrainSubsystem, Command command) {
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
    public void end(boolean interrupted) {
        this.command.end(interrupted || endedEarly);
    }
}
