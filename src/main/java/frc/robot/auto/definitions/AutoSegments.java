package frc.robot.auto.definitions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.auto.commands.AlignToGamePiece;
import frc.robot.auto.commands.AllianceFailsafe;
import frc.robot.auto.commands.AutoLevelCommand;
import frc.robot.auto.commands.FinetuneFieldPose;
import frc.robot.auto.commands.PickUpGamePiece;
import frc.robot.auto.commands.RunUntilNotLevel;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.auto.commands.WaitForVisionData;
import frc.robot.auto.util.AutoCommand;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoTemplate;
import frc.robot.commands.positions.HighNodeCommand;
import frc.robot.commands.positions.StowCommand;

public class AutoSegments {
    private RobotContainer container;
    public SendableChooser<AutoMode> chooser = new SendableChooser<AutoMode>();

    public AutoSegments(RobotContainer container) {
        this.container = container;
    }

    public AutoTemplate place_cube_high = new AutoTemplate(() -> {
        return new AutoCommand[] {
                AutoCommand.wrap(container.elevatorSubsystem.highNodeCommand(() -> true)),
                new SleepCommand(0.15),
                AutoCommand.wrap(container.telescopeSubsystem.highNodeCommand(() -> true)),
                new SleepCommand(0.5),
                AutoCommand.wrap(container.jointSubsystem.highNodeCommand(() -> true)),
                new SleepCommand(0.75),
                AutoCommand.wrap(container.clawSubsystem.openCommand()),
                new SleepCommand(0.25),
                AutoCommand.wrap(container.clawSubsystem.closeCommand()),
                AutoCommand.wrap(container.telescopeSubsystem.stowCommand()),
                AutoCommand.wrap(container.jointSubsystem.stowCommand()),
                new SleepCommand(0.4),
                AutoCommand.wrap(container.elevatorSubsystem.stowCommand())
        };
    });

    public AutoTemplate place_cone_high = new AutoTemplate(() -> {
        return new AutoCommand[] {
            AutoCommand.wrap(new HighNodeCommand(container.elevatorSubsystem, container.telescopeSubsystem, container.jointSubsystem, container.ledStripSubsystem, () -> false))
        };
    });

    public AutoTemplate stow = new AutoTemplate(() -> {
        return new AutoCommand[] {
            AutoCommand.wrap(new StowCommand(container.elevatorSubsystem, container.telescopeSubsystem, container.jointSubsystem))
        };
    });

    public AutoTemplate basic_mobility_1_3 = new AutoTemplate(() -> {
        return new AutoCommand[] {
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                    container.drivetrainSubsystem.zeroGyro();
                    container.drivetrainSubsystem.resetPose(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
                })),
                new FinetuneFieldPose(container.drivetrainSubsystem, new Pose2d(-4.136, 0, Rotation2d.fromRadians(0)),
                        -1),
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }))
        };
    });

    public AutoTemplate basic_balance_2 = new AutoTemplate(() -> {
        return new AutoCommand[] {
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                    container.drivetrainSubsystem.zeroGyro();
                    container.drivetrainSubsystem.resetPose(new Pose2d(0, 0, Rotation2d.fromRadians(0)));
                })),
                new RunUntilNotLevel(container.drivetrainSubsystem,
                        new FinetuneFieldPose(container.drivetrainSubsystem,
                                new Pose2d(-2, 0, Rotation2d.fromRadians(0)), -1)),
                new AutoLevelCommand(container.drivetrainSubsystem),
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                }))
        };
    });

    public AutoTemplate mobility_balance_blue_1_3 = new AutoTemplate(() -> {
        Pose2d station = new Pose2d(3.63, 2.81, Rotation2d.fromDegrees(180));
        Pose2d inFront = new Pose2d(5.68, 2.81, Rotation2d.fromDegrees(180));
        return new AutoCommand[] {
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                })),
                new WaitForVisionData(container.vision, 1.5),
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                })),
                new AllianceFailsafe(container.drivetrainSubsystem, AllianceFailsafe.Alliance.ALLIANCE_BLUE, false),
                new FinetuneFieldPose(container.drivetrainSubsystem, () -> {
                    return new Pose2d(inFront.getX(), container.drivetrainSubsystem.getPose().getY(),
                            inFront.getRotation());
                }, 0.15),
                new FinetuneFieldPose(container.drivetrainSubsystem, inFront, 0.15),
                new RunUntilNotLevel(container.drivetrainSubsystem,
                        new FinetuneFieldPose(container.drivetrainSubsystem, station, 0.15)),
                new AutoLevelCommand(container.drivetrainSubsystem)
        };
    });

    public AutoTemplate mobility_balance_red_1_3 = new AutoTemplate(() -> {
        Pose2d station = new Pose2d(12.44, 2.81, Rotation2d.fromDegrees(0));
        Pose2d inFront = new Pose2d(10.40, 2.81, Rotation2d.fromDegrees(0));
        return new AutoCommand[] {
            AutoCommand.wrap(new InstantCommand(() -> {
                container.vision.doOdometryUpdate = true;
            })),
            new WaitForVisionData(container.vision, 1.5),
            AutoCommand.wrap(new InstantCommand(() -> {
                container.vision.doOdometryUpdate = false;
            })),
            new AllianceFailsafe(container.drivetrainSubsystem, AllianceFailsafe.Alliance.ALLIANCE_RED, false),
            new FinetuneFieldPose(container.drivetrainSubsystem, () -> {
                return new Pose2d(inFront.getX(), container.drivetrainSubsystem.getPose().getY(),
                        inFront.getRotation());
            }, 0.15),
            new FinetuneFieldPose(container.drivetrainSubsystem, inFront, 0.15),
            new RunUntilNotLevel(container.drivetrainSubsystem,
                    new FinetuneFieldPose(container.drivetrainSubsystem, station, 0.15)),
            new AutoLevelCommand(container.drivetrainSubsystem)
        };
    });

    public AutoTemplate pickup_piece = new AutoTemplate(() -> {
        return new AutoCommand[] {
            new AlignToGamePiece(container.drivetrainSubsystem, container.vision, container.detector, 0),
            new PickUpGamePiece(container.drivetrainSubsystem, container.intakeSubsystem, container.jointSubsystem)
        };
    });
}
