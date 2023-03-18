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

    public static class DualAutoMode {
        public AutoMode blueMode;
        public AutoMode redMode;
    }

    // Auto mode definitions //

    public AutoMode mode_drive_colorless = new AutoMode(() -> {
        return new Command[] {
            new InstantCommand(() -> {
                container.vision.doOdometryUpdate = false;
                container.drivetrainSubsystem.zeroGyro();
                container.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
            }),
            new ApproximateFieldPose(container.trajectorySequencer, new Pose2d(2, 0, Rotation2d.fromRadians(0))),
            new FinetuneFieldPose(container.drivetrainSubsystem, new Pose2d(2, 0, Rotation2d.fromRadians(0))),
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new InstantCommand(() -> container.drivetrainSubsystem.resetGyro(Rotation2d.fromDegrees(180)))
        };
    });

    public AutoMode mode_balance_colorless = new AutoMode(() -> {
        return new Command[] {
            new InstantCommand(() -> {
                container.vision.doOdometryUpdate = false;
                container.drivetrainSubsystem.zeroGyro();
                container.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
            }),
            new RunUntilNotLevel(container.drivetrainSubsystem, FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, new Pose2d(2, 0, Rotation2d.fromRadians(0)))),
            new AutoLevelCommand(container.drivetrainSubsystem),
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new InstantCommand(() -> container.drivetrainSubsystem.resetGyro(Rotation2d.fromDegrees(180)))
        };
    });

    public AutoMode mode_mobility_balance_colorless = new AutoMode(() -> {
        return new Command[] {
            new InstantCommand(() -> {
                container.vision.doOdometryUpdate = false;
                container.drivetrainSubsystem.zeroGyro();
                container.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
            }),
            new ApproximateFieldPose(container.trajectorySequencer, new Pose2d(4.6, 0, Rotation2d.fromRadians(0))),
            new RunUntilNotLevel(container.drivetrainSubsystem, FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, new Pose2d(2, 0, Rotation2d.fromRadians(0)))),
            new AutoLevelCommand(container.drivetrainSubsystem),
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new InstantCommand(() -> container.drivetrainSubsystem.resetGyro(Rotation2d.fromDegrees(180)))
        };
    });

    public AutoMode mode_1_colorless = new AutoMode(() -> {
        return new Command[] {
            container.elevatorSubsystem.runHighNodeCommand(),
            new SleepCommand(1),
            container.telescopeSubsystem.runHighNodeCommand(),
            container.jointSubsystem.runHighNodeCommand(),
            new SleepCommand(1.5),
            container.clawSubsystem.openCommand(),
            new SleepCommand(1),
            container.clawSubsystem.closeCommand(),
            container.telescopeSubsystem.runStowCommand(),
            container.jointSubsystem.runStowCommand(),
            new SleepCommand(1),
            container.elevatorSubsystem.runStowCommand()
        };
    });

    public AutoMode mode_1_3_colorless = new AutoMode(() -> {
        return new Command[] {
            container.elevatorSubsystem.runHighNodeCommand(),
            new SleepCommand(1),
            container.telescopeSubsystem.runHighNodeCommand(),
            container.jointSubsystem.runHighNodeCommand(),
            new SleepCommand(1.5),
            container.clawSubsystem.openCommand(),
            new SleepCommand(1),
            container.clawSubsystem.closeCommand(),
            container.telescopeSubsystem.runStowCommand(),
            container.jointSubsystem.runStowCommand(),
            new SleepCommand(1),
            container.elevatorSubsystem.runStowCommand(),
            new SleepCommand(2),
            new InstantCommand(() -> {
                container.vision.doOdometryUpdate = false;
                container.drivetrainSubsystem.zeroGyro();
                container.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
            }),
            new RunUntilNotLevel(container.drivetrainSubsystem, FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, new Pose2d(2.5, 0, Rotation2d.fromRadians(0)))),
            new AutoLevelCommand(container.drivetrainSubsystem),
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new InstantCommand(() -> container.drivetrainSubsystem.resetGyro(Rotation2d.fromDegrees(180)))
        };
    });

    public AutoMode mode_1_2_3_colorless = new AutoMode(() -> {
        return new Command[] {
            container.elevatorSubsystem.runHighNodeCommand(),
            new SleepCommand(1),
            //container.telescopeSubsystem.runHighNodeCommand(),
            container.jointSubsystem.runHighNodeCommand(),
            new SleepCommand(1.5),
            container.clawSubsystem.openCommand(),
            new SleepCommand(1),
            container.clawSubsystem.closeCommand(),
            //container.telescopeSubsystem.runStowCommand(),
            container.jointSubsystem.runStowCommand(),
            new SleepCommand(1),
            container.elevatorSubsystem.runStowCommand(),
            new SleepCommand(2),
            new InstantCommand(() -> {
                container.vision.doOdometryUpdate = false;
                container.drivetrainSubsystem.zeroGyro();
                container.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
            }),
            new ApproximateFieldPose(container.trajectorySequencer, new Pose2d(4.6, 0, Rotation2d.fromRadians(0))),
            new RunUntilNotLevel(container.drivetrainSubsystem, FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, new Pose2d(2, 0, Rotation2d.fromRadians(0)))),
            new AutoLevelCommand(container.drivetrainSubsystem),
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;})        };
    });

    public AutoMode mode_station_blue = new AutoMode(() -> {
        Pose2d station = new Pose2d(3.63, 2.81, Rotation2d.fromDegrees(0));
        Pose2d inFront = new Pose2d(5.68, 2.81, Rotation2d.fromDegrees(0));
        return new Command[] {
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new WaitForVisionData(container.vision),
            new InstantCommand(() -> {container.vision.sampleRotation();}),
            FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, () -> {return new Pose2d(inFront.getX(), container.drivetrainSubsystem.getPoseMeters().getY(), inFront.getRotation());}),
            FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, inFront),
            new RunUntilNotLevel(container.drivetrainSubsystem, new ApproximateFieldPose(container.trajectorySequencer, station)),
            new AutoLevelCommand(container.drivetrainSubsystem)
        };
    });

    public AutoMode mode_station_red = new AutoMode(() -> {
        Pose2d station = new Pose2d(12.44, 2.81, Rotation2d.fromDegrees(180));
        Pose2d inFront = new Pose2d(10.40, 2.81, Rotation2d.fromDegrees(180));
        return new Command[] {
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new WaitForVisionData(container.vision),
            new InstantCommand(() -> {container.vision.sampleRotation();}),
            new ApproximateFieldPose(container.trajectorySequencer, new Pose2d()),
            FinetuneFieldPose.getPositioningCommand(container.trajectorySequencer, inFront),
            new RunUntilNotLevel(container.drivetrainSubsystem, new ApproximateFieldPose(container.trajectorySequencer, station)),
            new AutoLevelCommand(container.drivetrainSubsystem)
        };
    });

    public AutoMode test_mode = new AutoMode(() -> {
        return new Command[] {
            new InstantCommand(() -> {container.vision.doOdometryUpdate = true;}),
            new WaitForVisionData(container.vision),
            new InstantCommand(() -> {container.vision.sampleRotation();}),
        };
    });

    public void initAutonomous() {
        SmartDashboard.putData("Auto chooser", chooser);
        chooser.addOption("Charging station (blue)", mode_station_blue);
        chooser.addOption("Charging station (red)", mode_station_red);
        chooser.addOption("Mode \"2-3\" (colorless)", mode_mobility_balance_colorless);
        chooser.addOption("Mode \"1\" (colorless)", mode_1_colorless);
        chooser.addOption("Mode \"1-3\" (colorless)", mode_1_3_colorless);
        chooser.addOption("Mode \"1-2-3\" (colorless)", mode_1_2_3_colorless);
        chooser.addOption("Mode \"3\" (colorless)", mode_balance_colorless);
        chooser.addOption("Mode \"2\" (colorless)", mode_drive_colorless);
        chooser.addOption("LKSAJDFXHLMKFD", test_mode);
        chooser.setDefaultOption("No autonomous", null);
    }
}