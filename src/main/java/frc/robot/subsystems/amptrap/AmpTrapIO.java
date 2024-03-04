package frc.robot.subsystems.amptrap;

import org.littletonrobotics.junction.AutoLog;

public interface AmpTrapIO {
    @AutoLog
    public static class AmpTrapIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmperage = 0.0;
        public double temperatureCelsius = 0.0;
        public double velocityRotPerSecond = 0.0;
    }

    public default void updateInputs(AmpTrapIOInputs inputs) {
    }

    public default void setVelocity(double velocityRotPerSec) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setPercentOutput(double percentDecimal) {
    }

    public default void stop() {
        setVoltage(0.0);
    }
}
