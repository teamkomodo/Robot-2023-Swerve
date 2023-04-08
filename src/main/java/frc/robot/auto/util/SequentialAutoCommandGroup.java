package frc.robot.auto.util;

import edu.wpi.first.wpilibj2.command.Command;

public class SequentialAutoCommandGroup extends AutoCommand {
    private final AutoCommand[] commands;
    private int commandIndex = 0;

    private boolean failed = false;

    public SequentialAutoCommandGroup(AutoCommand... autoCommands) {
        for (Command c : autoCommands) {
            this.getRequirements().addAll(c.getRequirements());
        }
        commands = autoCommands;
    }

    @Override
    public void initialize() {
        commandIndex = 0;
        if (commands.length > 0) {
            commands[commandIndex].initialize();
        }
        failed = false;
    }

    @Override
    public void execute() {
        if (commandIndex >= commands.length) {
            return;
        }
        commands[commandIndex].execute();
        if (commands[commandIndex].isFinished()) {
            commands[commandIndex].end(false);
            if (!commands[commandIndex].didSucceed()) {
                failed = true;
                commandIndex = commands.length; // Terminate
                return;
            }
            commandIndex++;
            if (commandIndex < commands.length) {
                commands[commandIndex].initialize();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return commandIndex >= commands.length;
    }

    @Override
    public boolean didSucceed() {
        return !failed;
    }

    @Override
    public void end(boolean interrupted) {
        if (commandIndex < commands.length) {
            commands[commandIndex].end(interrupted);
        }
    }
}
