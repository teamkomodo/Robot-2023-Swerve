package frc.robot.commands.auto;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GetToToFDistance extends CommandBase {
    // public static Command getTotalToFCommand(DrivetrainSubsystem drivetrainSubsystem, TimeOfFlight sensor, double setDistance_meters) {
    //     return new SequentialCommandGroup(
    //         // new AlignToToF(drivetrainSubsystem, sensor),
    //         new GetToToFDistance(drivetrainSubsystem, sensor, setDistance_meters)
    //     );
    // }

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final TimeOfFlight sensor;
    private final PIDController controller;
    private final double setDistance;
    private final Timer correctTimer;
    public GetToToFDistance(DrivetrainSubsystem drivetrainSubsystem, TimeOfFlight sensor, double setDistance_meters) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.sensor = sensor;
        this.setDistance = setDistance_meters;
        this.correctTimer = new Timer();
        this.controller = new PIDController(2.4, 0.6, 0);
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize() {
        sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        sensor.setRangeOfInterest(8, 8, 12, 12);
        correctTimer.reset();
        correctTimer.start();
    }
    @Override
    public void execute() {
        double range = sensor.getRange() / 1000.0;
        SmartDashboard.putNumber("Range", range);
        if (!sensor.isRangeValid() || (sensor.getRangeSigma() > 15.0)) {
            drivetrainSubsystem.stopMotion();
            correctTimer.reset();
            return;
        }
        double forwardSpeed = this.controller.calculate(range, setDistance);
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(forwardSpeed, 0, 0));
        if (range - setDistance > AutoConstants.TOF_DISTANCE_TOLERANCE_METERS) {
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
