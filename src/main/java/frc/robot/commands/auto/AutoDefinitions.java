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
                new FinetuneFieldPose(container.drivetrainSubsystem, new Pose2d(2, 0, Rotation2d.fromRadians(0)), -1),
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }),
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
                new RunUntilNotLevel(container.drivetrainSubsystem,
                        new FinetuneFieldPose(container.drivetrainSubsystem,
                                new Pose2d(2, 0, Rotation2d.fromRadians(0)), -1)),
                new AutoLevelCommand(container.drivetrainSubsystem),
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }),
                new InstantCommand(() -> container.drivetrainSubsystem.resetGyro(Rotation2d.fromDegrees(180)))
        };
    });

    public AutoMode mode_station_blue = new AutoMode(() -> {
        Pose2d station = new Pose2d(3.63, 2.81, Rotation2d.fromDegrees(180));
        Pose2d inFront = new Pose2d(5.68, 2.81, Rotation2d.fromDegrees(180));
        return new Command[] {
                // Place cube
                container.elevatorSubsystem.highNodeCommand(() -> true),
                new SleepCommand(0.15),
                container.telescopeSubsystem.highNodeCommand(),
                new SleepCommand(0.5),
                container.jointSubsystem.highNodeCommand(),
                new SleepCommand(0.75),
                container.clawSubsystem.openCommand(),
                new SleepCommand(0.25),
                container.clawSubsystem.closeCommand(),
                container.telescopeSubsystem.stowCommand(),
                container.jointSubsystem.stowCommand(),
                new SleepCommand(0.4),
                container.elevatorSubsystem.stowCommand(),
                // Charging station
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }),
                new WaitForVisionData(container.vision),
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                }),
                new FinetuneFieldPose(container.drivetrainSubsystem, () -> {
                    return new Pose2d(inFront.getX(), container.drivetrainSubsystem.getPoseMeters().getY(),
                            inFront.getRotation());
                }, 0.15),
                new FinetuneFieldPose(container.drivetrainSubsystem, inFront, 0.15),
                new RunUntilNotLevel(container.drivetrainSubsystem,
                        new FinetuneFieldPose(container.drivetrainSubsystem, station, 0.15)),
                new AutoLevelCommand(container.drivetrainSubsystem)
        };
    });

    public AutoMode mode_station_red = new AutoMode(() -> {
        Pose2d station = new Pose2d(12.44, 2.81, Rotation2d.fromDegrees(0));
        Pose2d inFront = new Pose2d(10.40, 2.81, Rotation2d.fromDegrees(0));
        return new Command[] {
                // Place cube
                container.elevatorSubsystem.highNodeCommand(() -> true),
                new SleepCommand(0.15),
                container.telescopeSubsystem.highNodeCommand(),
                new SleepCommand(0.5),
                container.jointSubsystem.highNodeCommand(),
                new SleepCommand(0.75),
                container.clawSubsystem.openCommand(),
                new SleepCommand(0.25),
                container.clawSubsystem.closeCommand(),
                container.telescopeSubsystem.stowCommand(),
                container.jointSubsystem.stowCommand(),
                new SleepCommand(0.4),
                container.elevatorSubsystem.stowCommand(),
                // Charging station
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }),
                new WaitForVisionData(container.vision),
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                }),
                new FinetuneFieldPose(container.drivetrainSubsystem, () -> {
                    return new Pose2d(inFront.getX(), container.drivetrainSubsystem.getPoseMeters().getY(),
                            inFront.getRotation());
                }, 0.15),
                new FinetuneFieldPose(container.drivetrainSubsystem, inFront, 0.15),
                new RunUntilNotLevel(container.drivetrainSubsystem,
                        new FinetuneFieldPose(container.drivetrainSubsystem, station, 0.15)),
                new AutoLevelCommand(container.drivetrainSubsystem)
        };
    });

    public AutoMode test_mode = new AutoMode(() -> {
        return new Command[] {
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }),
                new WaitForVisionData(container.vision),
                new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                }),
                // new InstantCommand(() -> {
                // container.vision.sampleRotation();
                // }),
                new FinetuneFieldPose(container.drivetrainSubsystem, () -> {
                    return new Pose2d(3.63, 2.81, Rotation2d.fromDegrees(180));
                }, -1),
        };
    });

    public void initAutonomous() {
        SmartDashboard.putData("Auto chooser", chooser);
        chooser.addOption("Charging station (blue)", mode_station_blue);
        chooser.addOption("Charging station (red)", mode_station_red);
        // chooser.addOption("Mode \"2-3\" (colorless)",
        // mode_mobility_balance_colorless);
        // chooser.addOption("Mode \"1\" (colorless)", mode_1_colorless);
        // chooser.addOption("Mode \"1-3\" (colorless)", mode_1_3_colorless);
        // chooser.addOption("Mode \"1-2-3\" (colorless)", mode_1_2_3_colorless);
        chooser.addOption("Mode \"3\" (colorless)", mode_balance_colorless);
        chooser.addOption("Mode \"2\" (colorless)", mode_drive_colorless);
        chooser.addOption("LKSAJDFXHLMKFD", test_mode);
        chooser.setDefaultOption("No autonomous", null);
    }
}