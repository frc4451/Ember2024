package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    public double targetVelocityRotPerSecondLeft = 0;
    public double targetVelocityRotPerSecondRight = 0;

    @AutoLog
    public static class ShooterIOInputs {
        public double appliedVoltageLeft = 0.0;
        public double appliedVoltageRight = 0.0;

        public double currentAmperageLeft = 0.0;
        public double currentAmperageRight = 0.0;

        public double temperatureCelsiusLeft = 0.0;
        public double temperatureCelsiusRight = 0.0;

        public double velocityRotPerSecondLeft = 0.0;
        public double velocityRotPerSecondRight = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVelocity(
            double velocityRotPerSecondLeft,
            double velocityRotPerSecondRight) {
    }

    public default void setVoltage(double voltageLeft, double voltageRight) {
    }

    public default void stop() {
        setVoltage(0.0, 0.0);
    }
}
