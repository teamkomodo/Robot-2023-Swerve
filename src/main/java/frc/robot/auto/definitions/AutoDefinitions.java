package frc.robot.auto.definitions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.auto.commands.FollowSpecifiedPath;
import frc.robot.auto.commands.WaitForVisionData;
import frc.robot.auto.util.AutoCommand;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoSegment;
import frc.robot.auto.util.AutoTemplate;

@SuppressWarnings("unused")
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
        return AutoSegment.generateUnconditionalSequence(templates.place_cube_high,
                templates.mobility_balance_blue_1_3);
    });

    private AutoMode cube_mobility_balance_1_3_red = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.place_cube_high, templates.mobility_balance_red_1_3);
    });

    private AutoMode cube_balance_2_cl = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.place_cube_high, templates.basic_balance_2);
    });

    private AutoMode cube_mobility_1_3_cl = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.place_cube_high, templates.basic_mobility_1_3);
    });

    private AutoMode balance_2_cl = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.basic_balance_2);
    });

    private AutoMode mobility_1_3_cl = new AutoMode(() -> {
        return AutoSegment.generateUnconditionalSequence(templates.basic_mobility_1_3);
    });

    private AutoMode test_mode = new AutoMode(() -> {
        return new AutoSegment(new AutoTemplate(() -> {
            return new AutoCommand[] {
                    // AutoCommand.wrap(new InstantCommand(() -> {
                    // container.vision.doOdometryUpdate = false;
                    // container.drivetrainSubsystem.zeroGyro();
                    // container.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0,
                    // Rotation2d.fromRadians(0)));
                    // })),
                    AutoCommand.wrap(new InstantCommand(() -> {
                        container.vision.doOdometryUpdate = true;
                        container.vision.resetAccumulators();
                    })),
                    new WaitForVisionData(container.vision, 1.5),
                    AutoCommand.wrap(new InstantCommand(() -> {
                        container.vision.doOdometryUpdate = false;
                    })),
                    FollowSpecifiedPath.generatePathCommand(container.drivetrainSubsystem, container.sccf, false,
                            "testpath")
            };
        }));
    });

    public void initAutonomous() {
        SmartDashboard.putData("Auto chooser", chooser);
        chooser.addOption("Cube mobility balance (1,3) (blue)", cube_mobility_balance_1_3_blue);
        chooser.addOption("Cube mobility balance (1,3) (red)", cube_mobility_balance_1_3_red);
        chooser.addOption("Cube balance (2) (colorless)", cube_balance_2_cl);
        chooser.addOption("Cube mobility (1,3) (colorless)", cube_mobility_1_3_cl);
        chooser.addOption("Just balance (2) (colorless)", balance_2_cl);
        chooser.addOption("Just mobility (1,3) (colorless)", mobility_1_3_cl);
        chooser.addOption("Testing mode (DO NOT SELECT IN MATCH)", test_mode);
        chooser.setDefaultOption("No autonomous", null);
    }
}