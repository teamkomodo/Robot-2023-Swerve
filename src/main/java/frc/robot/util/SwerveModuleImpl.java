package frc.robot.util;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModuleImpl implements SwerveModule {
    private final SwerveModule backendSwerveModule;
    private double driveIntegral = 0.0;
    private long prev_t = -1;
    private double maxvel;
    private double maxvolt;

    private double prev_speedMetersPerSecond;
    private double prev_steerAngle;

    public SwerveModuleImpl(SwerveModule wrapped, double max_vel, double max_voltage) {
        backendSwerveModule = wrapped;
        maxvel = max_vel;
        maxvolt = max_voltage;
    }

    @Override
    public void set(double speedMetersPerSecond, double steerAngle) {
        backendSwerveModule.set(speedMetersPerSecond / maxvel * maxvolt, steerAngle);
        prev_speedMetersPerSecond = speedMetersPerSecond;
        prev_steerAngle = steerAngle;
    }

    @Override
    public double getSteerAngle() {
        if (RobotBase.isReal()) {
            return backendSwerveModule.getSteerAngle();
        }
        return prev_steerAngle;
    }

    @Override
    public double getDriveVelocity() {
        if (RobotBase.isReal()) {
            return backendSwerveModule.getDriveVelocity();
        }
        return prev_speedMetersPerSecond;
    }

    // Update drive integral
    public void periodic() {
        long curr_t = System.nanoTime() / 1000;
        if (prev_t != -1) {
            double dt = ((double) (curr_t - prev_t)) / 1000000.0; // Seconds
            driveIntegral += this.getDriveVelocity() * dt;
        }
        prev_t = curr_t;
    }

    public double getDrivePosition() {
        return driveIntegral;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getDrivePosition(), Rotation2d.fromDegrees(this.getSteerAngle()));
    }
}