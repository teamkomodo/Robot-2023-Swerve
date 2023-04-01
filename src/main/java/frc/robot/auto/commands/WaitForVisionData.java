package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.VisionPositioningSubsystem;

public class WaitForVisionData extends AutoCommand {
    private final VisionPositioningSubsystem vision;
    private final Timer timer = new Timer();
    private final double timeout;

    private int gotVisionData = 0;

    private Runnable prevOnVisionData = null;

    public WaitForVisionData(VisionPositioningSubsystem vision, double timeout) {
        this.vision = vision;
        this.gotVisionData = 0;
        this.timeout = timeout;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        prevOnVisionData = vision.getOnVisionData();
        vision.setOnVisionData(() -> {
            this.gotVisionData++;
        });
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Do nothing, wait for vision data
    }

    @Override
    public boolean isFinished() {
        if (timeout > 0) {
            if (timer.hasElapsed(this.timeout)) {
                return true;
            }
        }
        return gotVisionData >= 3 || RobotBase.isSimulation();
    }

    @Override
    public boolean didSucceed() {
        return gotVisionData >= 3 || RobotBase.isSimulation();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        vision.setOnVisionData(prevOnVisionData);
    }
}
