package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoDefinitions {
    private RobotContainer container;
    public SendableChooser<AutoMode> chooser = new SendableChooser<AutoMode>();

    public AutoDefinitions(RobotContainer container) {
        this.container = container;
    }

    public static class AutoMode {
        private final Supplier<Command[]> factory;

        public AutoMode(Supplier<Command[]> factory) {
            this.factory = factory;
        }

        public SequentialCommandGroup generateCommand() {
            return new SequentialCommandGroup(factory.get());
        }
    }

    // Auto mode definitions //

    public AutoMode mode_template = new AutoMode(() -> {
        Pose2d target = new Pose2d(5, 5, new Rotation2d(0));
        return new Command[] {
                new ApproximateFieldPose(container.trajectorySequencer, target),
                new FinetuneFieldPose(container.drivetrainSubsystem, target),
                new InstantCommand(() -> System.out.println("Done")),
        };
    });

    public void initAutonomous() {
        SmartDashboard.putData("Auto chooser", chooser);
        chooser.setDefaultOption("Template mode", mode_template);
    }
}