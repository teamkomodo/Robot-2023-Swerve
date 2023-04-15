package frc.robot.auto.commands;

import frc.robot.auto.util.AutoCommand;

public class CommandFailureTest extends AutoCommand {
    public CommandFailureTest() {
        // Do nothing
    }
    @Override
    public void initialize() {
        // Do not do anything
    }
    @Override
    public void execute() {
        // Carry out nothingness
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        // Perform an absolute lack of concrete actions
    }
    @Override
    public boolean didSucceed() {
        return false; // Failure test
    }
}
