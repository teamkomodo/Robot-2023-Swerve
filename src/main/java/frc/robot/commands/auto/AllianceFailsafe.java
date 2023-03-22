package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AllianceFailsafe extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Alliance alliance;
    public static enum Alliance {
        ALLIANCE_BLUE,
        ALLIANCE_RED
    }
    private boolean isOk = false;
    public AllianceFailsafe(DrivetrainSubsystem drivetrainSubsystem, Alliance target) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.alliance = target;
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize() {
        switch (alliance) {
        case ALLIANCE_BLUE:
            this.isOk = this.drivetrainSubsystem.getPoseMeters().getX() < 8.00;
        case ALLIANCE_RED:
            this.isOk = this.drivetrainSubsystem.getPoseMeters().getX() > 8.00;
        }
    }
    @Override
    public void execute() {
        // Do nothing
    }
    @Override
    public boolean isFinished() {
        return this.isOk;
    }
    @Override
    public void end(boolean interrupted) {

    }
}
