package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.util.AutoCommand;

public class SleepCommand extends AutoCommand {
    private final Timer timer = new Timer();
    private final double seconds;
    public SleepCommand(double seconds) {
        this.seconds = seconds;
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {}
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
