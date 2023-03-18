package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveDrivePoseEstimatorImpl;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModuleImpl;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;
    private double simGyroYawRadians = 0.0;
    private final HolonomicDriveController driveController;

    private boolean slowMode = true;

    public HolonomicDriveController getDriveController() {
        return driveController;
    }

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
            * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
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

    private final SwerveDrivePoseEstimatorImpl odometry;
    private final Field2d field2d;

    public DrivetrainSubsystem(Field2d field) {
        this.field2d = field;
        tab = Shuffleboard.getTab("Drivetrain");
        tab.addString("Chassis Speeds", () -> ("" + currentChassisSpeeds));

        tab.addDouble("Gyro Yaw", () -> (getGyroYaw().getDegrees()));

        frontLeftModule = new SwerveModuleImpl(createCustomNeo(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4iSwerveModuleHelper.GearRatio.L1,
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

        frontRightModule = new SwerveModuleImpl(createCustomNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        backLeftModule = new SwerveModuleImpl(createCustomNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        backRightModule = new SwerveModuleImpl(createCustomNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET),
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_VOLTAGE);

        odometry = new SwerveDrivePoseEstimatorImpl(
                kinematics, this.getGyroYaw(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
        odometry.getOdometry().forgetGyro = false;

        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.P_THETA_CONTROLLER,
                AutoConstants.I_THETA_CONTROLLER, 0,
                AutoConstants.THETA_PID_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.driveController = new HolonomicDriveController(
                new PIDController(AutoConstants.P_X_CONTROLLER, AutoConstants.I_X_CONTROLLER, 0),
                new PIDController(AutoConstants.P_X_CONTROLLER, AutoConstants.I_Y_CONTROLLER, 0),
                thetaController);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    private Rotation2d gyroOffset = Rotation2d.fromRadians(0);
    public void resetGyro(Rotation2d offset) {
        if (RobotBase.isReal()) {
            gyroOffset = getGyroYawRaw();
            gyroOffset = gyroOffset.minus(offset);
            return;
        }
        simGyroYawRadians = 0.0;
    }
    public void zeroGyro() {
        resetGyro(Rotation2d.fromRadians(0));
    }

    /**
     * 
     * @return a {@link Rotation2d} object with the heading of the robot (clockwise positive)
     */
    public Rotation2d getGyroYaw() {
        return getGyroYawRaw().minus(gyroOffset);
    }
    private Rotation2d getGyroYawRaw() {
        if (RobotBase.isReal()) {
            if (navx.isMagnetometerCalibrated()) {
                return Rotation2d.fromDegrees(navx.getFusedHeading());
            }
            return Rotation2d.fromDegrees(navx.getYaw());
        }
        return Rotation2d.fromRadians(simGyroYawRadians);
    }

    public void stopMotion() {
        this.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
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

    public void drive(double forward, double right, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(forward * (slowMode? DRIVETRAIN_SLOW_MODE_MODIFIER : 1), right * (slowMode? DRIVETRAIN_SLOW_MODE_MODIFIER : 1), rotation * (slowMode? DRIVETRAIN_SLOW_MODE_MODIFIER : 1));
        if (fieldRelative) {
            setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw().times(-1)));
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

    private SwerveModule createCustomNeo(
        ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        Mk4ModuleConfiguration configuration = new Mk4ModuleConfiguration();
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                new NeoDriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build(),
                new NeoSteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(SWERVE_STEER_P, SWERVE_STEER_I, SWERVE_STEER_D)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build())
        ).create(
                container,
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                )
        );
    }
    public Pose2d getPoseMeters() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d pose) {
        odometry.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }
    private Pose2d lastSimFieldPose = null;
    @Override
    public void periodic() {
        frontLeftModule.periodic();
        frontRightModule.periodic();
        backLeftModule.periodic();
        backRightModule.periodic();

        odometry.update(this.getGyroYaw(), getModulePositions());
        simGyroYawRadians += odometry.getOdometry().getLast_dtheta();
        if (RobotBase.isSimulation()) {
            if (lastSimFieldPose != null && !field2d.getRobotPose().equals(lastSimFieldPose)) {
                resetOdometry(field2d.getRobotPose());
            }
        }
        field2d.setRobotPose(this.getPoseMeters());
        if (RobotBase.isSimulation()) {
            lastSimFieldPose = field2d.getRobotPose();
        }
        drivePeriodic();
    }

    public Command runDisableSlowModeCommand() {
        return this.runOnce(() -> slowMode = false);
    }

    public Command runEnableSlowModeCommand() {
        return this.runOnce(() -> slowMode = true);
    }
}
