package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
        controller = new PIDController(AutoConstants.P_PIECE_LINEUP, AutoConstants.I_PIECE_LINEUP,
                AutoConstants.D_PIECE_LINEUP);
        addRequirements(drivetrainSubsystem);
        Shuffleboard.getTab("Piece alignment").add(controller);
    }

    @Override
    public void initialize() {
        detector.setEnabled(true);
        detector.setOneshot(false);
        correctTime.start();
    }

    @Override
    public void execute() {
        if (!detector.isReady()) {
            drivetrainSubsystem.stopMotion();
            correctTime.reset();
            return;
        }
        Target[] targets = detector.getAllTargets();
        if (targets == null || targets.length == 0) {
            drivetrainSubsystem.stopMotion();
            correctTime.reset();
            return;
        }
        Arrays.sort(targets, new Comparator<Target>() {
            @Override
            public int compare(Target t1, Target t2) {
                if (t1 == null) {
                    return -1;
                } else if (t2 == null) {
                    return 1;
                }
                return ((Double) t1.getArea()).compareTo(t2.getArea());
            }
        });
        Target bestTarget = null;
        int smallestIndex = 0;
        for (; smallestIndex < targets.length && (targets[smallestIndex] == null
                || targets[smallestIndex].getArea() < (targets[targets.length - 1].getArea() * 0.5)); smallestIndex++) {
        }
        for (int i = smallestIndex; i < targets.length; i++) {
            if (bestTarget == null || Math.abs(targets[i].centerX) < Math.abs(bestTarget.centerX)) {
                bestTarget = targets[i];
            }
        }
        if (bestTarget == null || !bestTarget.isValid()) {
            drivetrainSubsystem.stopMotion();
            correctTime.reset();
            return;
        }
        double xcomp = bestTarget.centerX;
        double angvel = controller.calculate(xcomp, targetX);
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, angvel));
        if (Math.abs(xcomp - targetX) > bestTarget.width * 0.5 * AutoConstants.MAX_PIECE_OFFSET_RATIO) {
            // Not lined up
            correctTime.reset();
            return;
        }
    }

    @Override
    public boolean isFinished() {
        // return false;
        return correctTime.hasElapsed(AutoConstants.PIECE_LINEUP_MIN_ALIGNMENT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        detector.setEnabled(false);
        correctTime.stop();
        drivetrainSubsystem.stopMotion();
    }
}
