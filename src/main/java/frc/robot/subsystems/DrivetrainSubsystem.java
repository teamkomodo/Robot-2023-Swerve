package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveDriveOdometryImpl;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModuleImpl;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;
    private double simGyroYawRadians = 0.0;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    public SwerveDriveKinematics getDriveKinematics() {
        return kinematics;
    }
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L1.getDriveReduction() * SdsModuleConfigurations.MK4_L1.getWheelDiameter()
            * Math.PI;
    private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    public AHRS getNavx() {
        return navx;
    }
    private final SwerveModuleImpl frontLeftModule;
    private final SwerveModuleImpl frontRightModule;
    private final SwerveModuleImpl backLeftModule;
    private final SwerveModuleImpl backRightModule;
    private final ShuffleboardTab tab;

    private final SwerveDriveOdometryImpl odometry;
    private final Field2d field2d;

    public DrivetrainSubsystem(Field2d field) {
        this.field2d = field;
        tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = new SwerveModuleImpl(Mk3SwerveModuleHelper.createNeo(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                // This is the ID of the drive motor
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        frontRightModule = new SwerveModuleImpl(Mk3SwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        backLeftModule = new SwerveModuleImpl(Mk3SwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        backRightModule = new SwerveModuleImpl(Mk3SwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        odometry = new SwerveDriveOdometryImpl(
                kinematics, this.getGyroYaw(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public void zeroGyro() {
        if (RobotBase.isReal()) {
            navx.zeroYaw();
            return;
        }
        simGyroYawRadians = 0.0;
    }

    public Rotation2d getGyroYaw() {
        if (RobotBase.isReal()) {
            if (navx.isMagnetometerCalibrated()) {
                return Rotation2d.fromDegrees(navx.getFusedHeading());
            }
            return Rotation2d.fromDegrees(360.0 - navx.getYaw());
        }
        return Rotation2d.fromRadians(simGyroYawRadians);
    }

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.set(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    private ChassisSpeeds currentChassisSpeeds = null;

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        currentChassisSpeeds = speeds;
    }

    public void drive(double forward, double right, double rotation, boolean fieldRealative) {
        ChassisSpeeds speeds = new ChassisSpeeds(forward, right, rotation);
        if(fieldRealative) {
            setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw()));
            return;
        }
        setChassisSpeeds(speeds);
    }

    private void drivePeriodic() {
        if (currentChassisSpeeds == null) {
            return;
        }
        setSwerveModuleStates(kinematics.toSwerveModuleStates(currentChassisSpeeds));
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        frontLeftModule.periodic();
        frontRightModule.periodic();
        backLeftModule.periodic();
        backRightModule.periodic();

        odometry.update(this.getGyroYaw(), getModulePositions());
        simGyroYawRadians += odometry.getLast_dtheta();
        field2d.setRobotPose(this.getPoseMeters());
        // setChassisSpeeds(new ChassisSpeeds(2, 0, 1));
        drivePeriodic();
    }
}
