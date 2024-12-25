// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnAngularOffsetPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVoltage = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTemperatureCelsius = 0.0;

        public SwerveModuleState state = new SwerveModuleState();
        public SwerveModulePosition position = new SwerveModulePosition();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public double[] odometryTurnPositionsRad = new double[] {};
        public SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public default void setDesiredState(SwerveModuleState desiredState) {
    }

    default SwerveModulePosition[] calculateOdometryPositions(SwerveModuleIOInputs inputs) {
        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        SwerveModulePosition[] odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            Rotation2d angle = new Rotation2d(inputs.odometryTurnPositionsRad[i]);
            odometryPositions[i] = new SwerveModulePosition(inputs.odometryDrivePositionsMeters[i], angle);
        }
        return odometryPositions;
    }

    public default void stop() {
    }
}
