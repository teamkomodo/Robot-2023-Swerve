package frc.robot.auto.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoMode {
    private Supplier<AutoSegment> factory;
    public AutoMode(Supplier<AutoSegment> factory) {
        this.factory = factory;
    }
    public Command generateAutonomousEngine(RobotContainer container) {
        return new AutoEngine(factory.get(), container);
    }
}
