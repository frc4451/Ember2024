package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double[] appliedVoltage = { 0.0, 0.0 };
        public double[] currentAmperage = { 0.0, 0.0 };
        public double[] temperatureCelsius = { 0.0, 0.0 };
        public double[] velocityRotPerSecond = { 0.0, 0.0 };
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
    }

    public default void setFree() {
    }

    public default void stop() {
        setVelocity(0.0, 0.0);
    }
}
