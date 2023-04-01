package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.VisionPositioningSubsystem;

public class WaitForVisionData extends AutoCommand {
    private final VisionPositioningSubsystem vision;

    private int gotVisionData = 0;
    public WaitForVisionData(VisionPositioningSubsystem vision) {
        this.vision = vision;
        this.gotVisionData = 0;
        addRequirements(vision);
    }
    @Override
    public void initialize() {
        vision.setOnVisionData(() -> {
            this.gotVisionData++;
        });
    }
    @Override
    public void execute() {
        // Do nothing, wait for vision data
    }
    @Override
    public boolean isFinished() {
        return gotVisionData >= 3 || RobotBase.isSimulation();
    }
    @Override
    public void end(boolean interrupted) {
        vision.setOnVisionData(null);
    }
}
