package frc.robot.commands.auto;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetToToFDistance extends CommandBase {
    // public static Command getTotalToFCommand(DrivetrainSubsystem drivetrainSubsystem, TimeOfFlight sensor, double setDistance_meters) {
    //     return new SequentialCommandGroup(
    //         // new AlignToToF(drivetrainSubsystem, sensor),
    //         new GetToToFDistance(drivetrainSubsystem, sensor, setDistance_meters)
    //     );
    // }

    private double clamp(double val, double mag) {
        if (val > mag) {
            return mag;
        }
        if (val < -mag) {
            return -mag;
        }
        return val;
    }

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final TimeOfFlight sensor;
    private final PIDController controller;
    private final double setDistance;
    private final Timer correctTimer;

    private double getFPGATimestamp() {
        return RobotController.getFPGATime() / 1000000.0;
    }

    private double range_accumulator;
    private double accumulator;
    public GetToToFDistance(DrivetrainSubsystem drivetrainSubsystem, TimeOfFlight sensor, double setDistance_meters) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.sensor = sensor;
        this.setDistance = setDistance_meters;
        this.correctTimer = new Timer();
        this.controller = new PIDController(3.5, 1.0, 0);
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize() {
        sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        sensor.setRangeOfInterest(8, 8, 12, 12);
        correctTimer.reset();
        correctTimer.start();
        this.range_accumulator = 0.0;
        this.accumulator = 0.0;
    }
    @Override
    public void execute() {
        double range = sensor.getRange() / 1000.0;
        SmartDashboard.putNumber("Range", range);
        if (!sensor.isRangeValid() || (sensor.getRangeSigma() > 20.0)) {
            drivetrainSubsystem.stopMotion();
            correctTimer.reset();
            return;
        }
        
        this.range_accumulator += range;
        this.accumulator += 1.0;
        this.range_accumulator *= AutoConstants.TOF_LEAKY_COEFFICIENT;
        this.accumulator *= AutoConstants.TOF_LEAKY_COEFFICIENT;
        range = this.range_accumulator / this.accumulator;
        double forwardSpeed = this.controller.calculate(range, setDistance - AutoConstants.TOF_DISTANCE_TOLERANCE_METERS);
        forwardSpeed = clamp(forwardSpeed, 0.45);
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(forwardSpeed, 0, 0));
        if (range > setDistance) {
            correctTimer.reset();
        }
    }
    @Override
    public boolean isFinished() {
        return correctTimer.hasElapsed(AutoConstants.TOF_DISTANCE_ELAPSED_SECONDS);
    }
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMotion();
    }
}
