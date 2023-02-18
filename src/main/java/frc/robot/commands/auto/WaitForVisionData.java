package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionPositioningSubsystem;

public class WaitForVisionData extends CommandBase {
    private final VisionPositioningSubsystem vision;

    private boolean gotVisionData = false;
    public WaitForVisionData(VisionPositioningSubsystem vision) {
        this.vision = vision;
        addRequirements(vision);
    }
    @Override
    public void initialize() {
        vision.setOnVisionData(() -> {
            this.gotVisionData = true;
        });
    }
    @Override
    public void execute() {
        // Do nothing, wait for vision data
    }
    @Override
    public boolean isFinished() {
        return gotVisionData;
    }
    @Override
    public void end(boolean interrupted) {

    }
}
