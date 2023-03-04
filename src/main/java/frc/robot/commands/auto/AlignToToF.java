package frc.robot.commands.auto;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignToToF extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final TimeOfFlight sensor;

    public AlignToToF(DrivetrainSubsystem drivetrainSubsystem, TimeOfFlight sensor) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.sensor = sensor;
        addRequirements(drivetrainSubsystem);
    }

    private double range = 0.0;
    private double weight = 0.0;
    private int stage = 0;
    private long count = 0;
    private Rotation2d prevRot = null;
    private Rotation2d bestRot = null;
    private double bestRange;

    @Override
    public void initialize() {
        this.sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        this.sensor.setRangeOfInterest(8, 8, 12, 12);
    }

    @Override
    public void execute() {
        double r = sensor.getRange();
        if (sensor.isRangeValid() && sensor.getRangeSigma() <= 15.0) {
            range += r;
            weight += 1;
            range *= AutoConstants.TOF_LEAKY_COEFFICIENT;
            weight *= AutoConstants.TOF_LEAKY_COEFFICIENT;
        }

        double proc_range = weight <= 0.0 ? 2000 : (range / weight);

        switch (stage) {
            case 0:
                prevRot = drivetrainSubsystem.getPoseMeters().getRotation();
                bestRot = drivetrainSubsystem.getPoseMeters().getRotation();
                bestRange = proc_range;
                if (count > 10) {
                    stage++;
                }
                count++;
                break;
            case 1: {
                drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, AutoConstants.TOF_ANGULAR_VELOCITY));
                if (proc_range < bestRange) {
                    bestRot = drivetrainSubsystem.getPoseMeters().getRotation();
                    bestRange = proc_range;
                }
                double offset = drivetrainSubsystem.getPoseMeters().getRotation().minus(prevRot).getRadians()
                        % (2 * Math.PI);
                if (Math.abs(bestRange - proc_range) > 100) {
                    if (Math.abs(drivetrainSubsystem.getPoseMeters().getRotation().minus(prevRot).getRadians()) < Math
                            .toRadians(5)) {
                        stage++;
                        break;
                    }
                }
                if (offset >= AutoConstants.TOF_HALF_SWEEP_ANGLE
                        && offset <= AutoConstants.TOF_HALF_SWEEP_ANGLE + (Math.PI * 0.5)) {
                    stage++;
                }
            }
                break;
            case 2: {
                drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, -AutoConstants.TOF_ANGULAR_VELOCITY));
                if (proc_range < bestRange) {
                    bestRot = drivetrainSubsystem.getPoseMeters().getRotation();
                    bestRange = proc_range;
                }
                double offset = drivetrainSubsystem.getPoseMeters().getRotation().minus(prevRot).getRadians();
                if (offset > 0) {
                    offset -= 2 * Math.PI;
                }
                offset %= 2 * Math.PI;
                if (Math.abs(bestRange - proc_range) > 100 && offset <= 0
                        && offset >= -AutoConstants.TOF_HALF_SWEEP_ANGLE) {
                    if (Math.abs(drivetrainSubsystem.getPoseMeters().getRotation().minus(prevRot).getRadians()) < Math
                            .toRadians(5)) {
                        stage++;
                        for (int i = 0; i < 10; i++) {
                            System.out.println("SDAKFJHMKSDJHFC " + bestRange);
                        }
                        break;
                    }
                }
                if (offset <= -AutoConstants.TOF_HALF_SWEEP_ANGLE
                        && offset >= -(AutoConstants.TOF_HALF_SWEEP_ANGLE + (Math.PI * 0.5))) {
                    stage++;
                    for (int i = 0; i < 10; i++) {
                        System.out.println("SDAKFJHMKSDJHFC " + bestRange);
                    }
                }
            }
                break;
            case 3:
                ChassisSpeeds speeds = drivetrainSubsystem.getDriveController().calculate(
                        drivetrainSubsystem.getPoseMeters(),
                        new Pose2d(drivetrainSubsystem.getPoseMeters().getTranslation(), bestRot), 0, bestRot);
                drivetrainSubsystem.setChassisSpeeds(speeds);
                break;
        }
        // SmartDashboard.putNumber("Processed range", proc_range);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMotion();
    }
}
