package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.VisionPipelineConnector;
import frc.robot.util.VisionPipelineConnector.Target;

public class AlignToGamePiece extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final VisionPipelineConnector detector;
    private final Timer correctTime = new Timer();
    private final double targetX;
    private final PIDController controller;

    // Use targetX = 0 for straight alignment
    public AlignToGamePiece(DrivetrainSubsystem drivetrainSubsystem, VisionPipelineConnector detector,
            double targetX) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.detector = detector;
        this.targetX = targetX;
        controller = new PIDController(AutoConstants.P_PIECE_LINEUP, AutoConstants.I_PIECE_LINEUP, 0);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        // Do nothing
        correctTime.start();
    }

    @Override
    public void execute() {
        if (!detector.isReady()) {
            drivetrainSubsystem.stopMotion();
            correctTime.reset();
        }
        Target largestTarget = detector.getLargestTarget();
        if (largestTarget == null || !largestTarget.isValid()) {
            drivetrainSubsystem.stopMotion();
            correctTime.reset();
        }
        double xcomp = detector.getLargestTarget().centerX;
        double angvel = controller.calculate(xcomp, targetX);
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, angvel));
        if (Math.abs(xcomp - targetX) > AutoConstants.MAX_PIECE_OFFSET_IMGCOORD) {
            // Not lined up
            correctTime.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return correctTime.hasElapsed(AutoConstants.PIECE_LINEUP_MIN_ALIGNMENT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        correctTime.stop();
        drivetrainSubsystem.stopMotion();
    }
}
