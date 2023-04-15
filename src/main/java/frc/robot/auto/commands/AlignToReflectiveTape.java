package frc.robot.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.LimelightConnector;
import frc.robot.util.LimelightConnector.LimelightResult;

public class AlignToReflectiveTape extends AutoCommand {
    public static enum TapeLevel {
        LOW_TAPE,
        HIGH_TAPE
    }
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final LimelightConnector limelight;
    private final Timer correctTime = new Timer();
    private final Timer waitTime = new Timer();
    private final int pipelineIndex;
    private final PIDController alignmentController = new PIDController(AutoConstants.P_REFL_LINEUP, AutoConstants.I_REFL_LINEUP, AutoConstants.D_REFL_LINEUP);
    public AlignToReflectiveTape(DrivetrainSubsystem drivetrainSubsystem, LimelightConnector limelight, TapeLevel level) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.limelight = limelight;
        pipelineIndex = (level == TapeLevel.LOW_TAPE) ? 0 : 1;
        addRequirements(drivetrainSubsystem);
    }
    private double clamp(double val, double mag) {
        if (val > mag) {
            return mag;
        }
        if (val < -mag) {
            return -mag;
        }
        return val;
    }
    @Override
    public void initialize() {
        this.limelight.setLEDs(true);
        correctTime.reset();
        correctTime.start();
        waitTime.reset();
        waitTime.start();
        this.limelight.setPipeline(this.pipelineIndex);
    }
    @Override
    public void execute() {
        if (!waitTime.hasElapsed(0.65)) {
            correctTime.reset();
            drivetrainSubsystem.stopMotion();
            return;
        }
        LimelightResult result = this.limelight.get();
        if (result == null) {
            drivetrainSubsystem.stopMotion();
            correctTime.reset();
            return;
        }
        double angvel = alignmentController.calculate(result.tx / 27.0, 0);
        angvel = clamp(angvel, 0.8);
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, -angvel, 0));
        if (Math.abs(result.tx) > AutoConstants.MIN_REFLECTIVE_OFFSET_DEGREES) {
            correctTime.reset();
        }
    }
    @Override
    public boolean isFinished() {
        return correctTime.hasElapsed(AutoConstants.REFLECTIVE_MIN_ALIGN_TIME);
    }
    @Override
    public void end(boolean interrupted) {
        this.limelight.setLEDs(false);
        this.limelight.setPipeline(2);
        this.correctTime.stop();
    }
}
