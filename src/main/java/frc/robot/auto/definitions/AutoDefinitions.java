package frc.robot.auto.definitions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.auto.commands.FinetuneFieldPose;
import frc.robot.auto.commands.WaitForVisionData;
import frc.robot.auto.util.AutoCommand;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoSegment;
import frc.robot.auto.util.AutoTemplate;

public class AutoDefinitions {
    private RobotContainer container;
    public SendableChooser<AutoMode> chooser = new SendableChooser<AutoMode>();
    private AutoSegments templates;

    public AutoDefinitions(RobotContainer container) {
        this.templates = new AutoSegments(container);
        this.container = container;
    }

    // Auto mode definitions //
    private AutoMode cube_mobility_balance_1_3_blue = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.place_cube_high, templates.mobility_balance_blue_1_3);
    });

    private AutoMode cube_mobility_balance_1_3_red = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.place_cube_high, templates.mobility_balance_red_1_3);
    });

    private AutoMode test_mode = new AutoMode(() -> {
        return new AutoSegment(new AutoTemplate(() -> {
            Pose2d station = new Pose2d(3.63, 2.81, Rotation2d.fromDegrees(0));
            return new AutoCommand[] {
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = true;
                })),
                new WaitForVisionData(container.vision),
                AutoCommand.wrap(new InstantCommand(() -> {
                    container.vision.doOdometryUpdate = false;
                })),
                new FinetuneFieldPose(container.drivetrainSubsystem, station, 0.15),
            };
        }));
    });

    public void initAutonomous() {
        SmartDashboard.putData("Auto chooser", chooser);
        chooser.addOption("Cube mobility balance (1,3) (blue)", cube_mobility_balance_1_3_blue);
        chooser.addOption("Cube mobility balance (1,3) (red)", cube_mobility_balance_1_3_red);
        // chooser.addOption("Cube balance (2) (colorless)", mode_balance_colorless);
        // chooser.addOption("Cube mobility (1,3) (colorless)", mode_drive_colorless);
        chooser.addOption("Testing mode (DO_NOT_SELECT)", test_mode);
        chooser.setDefaultOption("No autonomous", null);
    }
}