package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SwerveControllerCommandFactory;
import frc.robot.commands.SwerveControllerCommandFactory.SwerveControllerCommandDescriptor;

public class TrajectorySequencer extends SubsystemBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final SwerveControllerCommandFactory controllerCommandFactory;
    private final Supplier<Pose2d> fieldPosTargetSupplier;
    private final Supplier<Transform2d> fieldVelTargetSupplier;
    private ArrayList<SwerveControllerCommandDescriptor> commandStack = new ArrayList<SwerveControllerCommandDescriptor>();

    public TrajectorySequencer(DrivetrainSubsystem drive, SwerveControllerCommandFactory controllerCommandFactory,
            Supplier<Pose2d> fieldPosTargetSupplier,
            Supplier<Transform2d> fieldVelTargetSupplier) {
        this.drivetrainSubsystem = drive;
        this.controllerCommandFactory = controllerCommandFactory;
        this.fieldPosTargetSupplier = fieldPosTargetSupplier;
        this.fieldVelTargetSupplier = fieldVelTargetSupplier;
    }

    public void startRelativeTrajectory(Pose2d initialPose, List<Translation2d> waypoints, Pose2d finalPose,
            boolean overwriteCurrentCommand) {
        SwerveControllerCommandDescriptor controllerDesc = controllerCommandFactory
                .generateCommand(TrajectoryGenerator
                        .generateTrajectory(initialPose, waypoints, finalPose,
                                controllerCommandFactory.getTrajectoryConfig()));
        if (overwriteCurrentCommand) {
            if (commandStack.size() > 0) {
                commandStack.get(0).command.end(true);
            }
            commandStack.clear();
        }
        commandStack.add(controllerDesc);
        if (commandStack.size() == 1) {
            controllerDesc.command.initialize();
        }
    }

    public void startRelativeTrajectory(Pose2d initialPose, List<Translation2d> waypoints, Pose2d finalPose) {
        startRelativeTrajectory(initialPose, waypoints, finalPose, false);
    }

    public void pause() {
        if (commandStack.size() > 0) {
            commandStack.get(0).baseCommand.pause();
        }
    }

    public void unpause() {
        if (commandStack.size() > 0) {
            commandStack.get(0).baseCommand.unpause();
        }
    }

    private double unchecked_getTargetDistanceMeters(double timeFromNow_sec) {
        double trajSampleTime = timeFromNow_sec + commandStack.get(0).baseCommand.getTimer().get();
        Pose2d samplePose = commandStack.get(0).trajectory.sample(trajSampleTime).poseMeters;
        Pose2d targetPosCurrent = fieldPosTargetSupplier.get();
        Transform2d targetVelCurrent = fieldVelTargetSupplier.get();
        Pose2d targetPosExtrapolated = targetPosCurrent.plus(targetVelCurrent.times(timeFromNow_sec));
        return samplePose.getTranslation().getDistance(targetPosExtrapolated.getTranslation());
    }

    private double unchecked_getTargetDistanceDerivativeMeters(double timeFromNow_sec) {
        return (unchecked_getTargetDistanceMeters(
                timeFromNow_sec + (AutoConstants.TARGET_INTERCEPT_DIFFERENTIAL_SECONDS / 2.0))
                - unchecked_getTargetDistanceMeters(
                        timeFromNow_sec - (AutoConstants.TARGET_INTERCEPT_DIFFERENTIAL_SECONDS / 2.0)))
                / AutoConstants.TARGET_INTERCEPT_DIFFERENTIAL_SECONDS;
    }

    private double getClosestTargetIntersectionMeters() {
        if (fieldPosTargetSupplier == null || fieldVelTargetSupplier == null) {
            return -1;
        }
        if (commandStack.size() == 0) {
            return -2;
        }
        double time = 0;
        double deriv = unchecked_getTargetDistanceDerivativeMeters(time);
        if (deriv > 0) {
            return unchecked_getTargetDistanceMeters(0.0);
        }
        int iterations = 0;
        while (deriv < 0.05) { // Relative target velocity is faster than 50 cm/s
            time -= AutoConstants.TARGET_INTERCEPT_GRAD_RATE * deriv;
            deriv = unchecked_getTargetDistanceDerivativeMeters(time);
            if (iterations > 100) {
                // Failsafe - make sure we don't block the main thread
                return -3;
            }
        }
        return unchecked_getTargetDistanceMeters(time);
    }

    long lastInterceptCheck = -1;

    @Override
    public void periodic() {
        if (AutoConstants.ENABLE_TARGET_INTERCEPT_CHECK) {
            if (lastInterceptCheck == (long) (-1)) {
                lastInterceptCheck = System.currentTimeMillis();
            } else if (System.currentTimeMillis()
                    - lastInterceptCheck > AutoConstants.TARGET_INTERCEPT_CHECK_PERIOD_MS) {
                double interceptDistanceMeters = getClosestTargetIntersectionMeters();
                if (interceptDistanceMeters > 0) {
                    if (interceptDistanceMeters < AutoConstants.MINIMUM_ALLOWABLE_TARGET_INTERSEPT_SEPARATION) {
                        this.pause();
                    } else {
                        this.unpause();
                    }
                }
                lastInterceptCheck = System.currentTimeMillis();
            }
        }
        if (commandStack.size() > 0) {
            commandStack.get(0).command.execute();
            if (commandStack.get(0).command.isFinished()) {
                commandStack.get(0).command.end(false);
                drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
                commandStack.remove(0);
                if (commandStack.size() > 0) {
                    commandStack.get(0).command.initialize();
                }
            }
        }
    }
}
