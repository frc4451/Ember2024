package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveGyroIO {
    @AutoLog
    public static class SwerveGyroIOInputs {
        public boolean isConnected = false;
        public double yawPositionRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;

        public double[] odometryYawTimestamps = new double[] {};
        public double[] odometryYawPositionsRad = new double[] {};
    }

    public default void updateInputs(SwerveGyroIOInputs inputs) {
    }

    public default void zero() {
    }
}
