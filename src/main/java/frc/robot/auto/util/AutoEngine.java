package frc.robot.auto.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoEngine extends CommandBase {
    private AutoSegment current = null;
    private final RobotContainer container;
    private SequentialAutoCommandGroup currentCommand = null;

    private void clearRequirementFlags(AutoSegment segment) {
        if (segment == null || !segment.requirementsAdded) {
            return;
        }
        segment.requirementsAdded = false;
        clearRequirementFlags(segment.onFailure);
        clearRequirementFlags(segment.onSuccess);
    }

    private void addAllRequirements(AutoSegment segment) {
        if (segment == null || segment.requirementsAdded) {
            return;
        }
        AutoCommand[] commands = segment.baseTemplate.getFactory().get();
        for (AutoCommand c : commands) {
            this.getRequirements().addAll(c.getRequirements());
        }
        segment.requirementsAdded = true;
        addAllRequirements(segment.onFailure);
        addAllRequirements(segment.onSuccess);
    }

    public AutoEngine(AutoSegment head, RobotContainer container) {
        this.current = head;
        this.container = container;
        clearRequirementFlags(head);
        addAllRequirements(head);
    }

    @Override
    public void initialize() {
        // Do nothing
    }

    @Override
    public void execute() {
        if (currentCommand == null) {
            if (current == null) {
                // Cannot proceed
                return;
            }
            // Generate command group
            currentCommand = current.baseTemplate.generateCommand(container);
        }
        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            if (currentCommand.didSucceed()) {
                current = current.onSuccess;
            } else {
                current = current.onFailure;
            }
            currentCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        return current == null && currentCommand == null;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }
}
