package frc.robot.auto.commands;

import frc.robot.auto.util.AutoCommand;

public class CommandFailureTest extends AutoCommand {
    public CommandFailureTest() {
        // Do nothing
    }
    @Override
    public void initialize() {
        // Do nothing
    }
    @Override
    public void execute() {
        // Do nothing
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        // Do nothing
    }
    @Override
    public boolean didSucceed() {
        return false; // Failure test
    }
}
