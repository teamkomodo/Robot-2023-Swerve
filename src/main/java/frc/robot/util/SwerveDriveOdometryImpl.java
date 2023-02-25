// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveDriveOdometryImpl {
    private final SwerveDriveKinematics m_kinematics;
    private Pose2d m_poseMeters;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    private final int m_numModules;
    private SwerveModulePosition[] m_previousModulePositions;

    public boolean forgetGyro = false;

    public SwerveDriveOdometryImpl(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPose) {
        m_kinematics = kinematics;
        m_poseMeters = initialPose;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPose.getRotation();
        m_numModules = modulePositions.length;

        m_previousModulePositions = new SwerveModulePosition[m_numModules];
        for (int index = 0; index < m_numModules; index++) {
            m_previousModulePositions[index] = new SwerveModulePosition(
                    modulePositions[index].distanceMeters, modulePositions[index].angle);
        }

        MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);
    }

    public SwerveDriveOdometryImpl(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions) {
        this(kinematics, gyroAngle, modulePositions, new Pose2d());
    }

    public void resetPosition(
            Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        if (modulePositions.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }

        m_poseMeters = pose;
        m_previousAngle = pose.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        for (int index = 0; index < m_numModules; index++) {
            m_previousModulePositions[index] = new SwerveModulePosition(
                    modulePositions[index].distanceMeters, modulePositions[index].angle);
        }
    }

    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }

    private double last_dtheta = 0;
    public double getLast_dtheta() {
        return last_dtheta;
    }
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        if (modulePositions.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var moduleDeltas = new SwerveModulePosition[m_numModules];
        for (int index = 0; index < m_numModules; index++) {
            var current = modulePositions[index];
            var previous = m_previousModulePositions[index];

            moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters,
                    current.angle);
            previous.distanceMeters = current.distanceMeters;
        }

        var angle = gyroAngle.plus(m_gyroOffset);

        var twist = m_kinematics.toTwist2d(moduleDeltas);
        last_dtheta = twist.dtheta;
        if (RobotBase.isReal() && !forgetGyro) {
            twist.dtheta = angle.minus(m_previousAngle).getRadians();
        }

        var newPose = m_poseMeters.exp(twist);

        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), (RobotBase.isReal() && !forgetGyro) ? angle : newPose.getRotation());

        return m_poseMeters;
    }
}
