// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveCurrentAmperage = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnAngularOffsetPositionRad = 0.0;
        public double turnAppliedVoltage = 0.0;
        public double turnCurrentAmperage = 0.0;
        // public double turnVelocityRadPerSec = 0.0;

        public SwerveModuleState state = new SwerveModuleState();
        public SwerveModulePosition position = new SwerveModulePosition();
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    public default void setDriveVoltage(double voltage) {
    }

    public default void setTurnVoltage(double voltage) {
    }

    public default void runCharacterization(double driveVoltage) {
    }

    public default void stop() {
        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public default void setDesiredState(SwerveModuleState desiredState) {
    }
}
