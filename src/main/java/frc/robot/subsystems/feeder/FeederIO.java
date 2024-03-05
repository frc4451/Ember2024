package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmperage = 0.0;
        public double temperatureCelsius = 0.0;

        public double velocityRotPerSecond = 0.0;
    }

    public default void updateInputs(FeederIOInputs inputs) {
    }

    public default void setVelocity(double velocityRotPerSecond) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stop() {
        setVoltage(0.0);
    }
}
