package frc.robot.auto.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class AutoCommand extends CommandBase {
    public static AutoCommand wrap(Command baseCommand) {
        return new AutoCommand() {
            @Override
            public boolean didSucceed() {
                return true;
            }

            @Override
            public void initialize() {
                baseCommand.initialize();
            }

            @Override
            public void execute() {
                baseCommand.execute();
            }

            @Override
            public boolean isFinished() {
                return baseCommand.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                baseCommand.end(interrupted);
            }
        };
    }

    public boolean didSucceed() {
        return isFinished();
    }
}
