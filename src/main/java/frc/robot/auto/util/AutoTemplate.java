package frc.robot.auto.util;

import java.util.ArrayList;
import java.util.function.Supplier;

import frc.robot.RobotContainer;

public class AutoTemplate {
    private final Supplier<AutoCommand[]> factory;

    public AutoTemplate(Supplier<AutoCommand[]> factory) {
        this.factory = factory;
    }

    public Supplier<AutoCommand[]> getFactory() {
        return factory;
    }

    public SequentialAutoCommandGroup generateCommand(RobotContainer container) {
        ArrayList<AutoCommand> commands = new ArrayList<>();
        AutoCommand[] source = factory.get();
        for (int i = 0; i < source.length; i++) {
            commands.add(AutoCommand.wrap(container.ledStripSubsystem.setSolidColor(i)));
            commands.add(source[i]);
        }
        return new SequentialAutoCommandGroup(commands.toArray(new AutoCommand[commands.size()]));
    }
}
