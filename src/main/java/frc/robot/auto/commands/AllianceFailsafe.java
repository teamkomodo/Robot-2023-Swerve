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
    private final boolean blockOnFail;

    public AllianceFailsafe(DrivetrainSubsystem drivetrainSubsystem, Alliance target, boolean blockOnFail) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.alliance = target;
        this.blockOnFail = blockOnFail;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        switch (alliance) {
            case ALLIANCE_BLUE:
                this.isOk = this.drivetrainSubsystem.getPose().getX() < 8.00;
                break;
            case ALLIANCE_RED:
                this.isOk = this.drivetrainSubsystem.getPose().getX() > 8.00;
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
        return this.isOk || !this.blockOnFail;
    }

    @Override
    public void end(boolean interrupted) {
        // Do nothing
    }

    @Override
    public boolean didSucceed() {
        return this.isOk;
    }
}
