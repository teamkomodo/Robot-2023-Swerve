package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AllianceFailsafe extends AutoCommand {
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
            break;
        case ALLIANCE_RED:
            this.isOk = this.drivetrainSubsystem.getPoseMeters().getX() > 8.00;
            break;
        }
        if (!this.isOk) {
            DriverStation.reportError("Fatal: Failed alliance check. Not running autonomous.", false);
            System.err.println("Fatal: Failed alliance check. Not running autonomous.");
        } else {
            System.out.println("Debug: Passed alliance safety check.");
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
