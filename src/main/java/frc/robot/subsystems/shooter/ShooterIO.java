package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double appliedVoltageLeft = 0.0;
        public double appliedVoltageRight = 0.0;
        public double appliedVoltageFeeder = 0.0;

        public double currentAmperageLeft = 0.0;
        public double currentAmperageRight = 0.0;
        public double currentAmperageFeeder = 0.0;

        public double temperatureCelsiusLeft = 0.0;
        public double temperatureCelsiusRight = 0.0;
        public double temperatureCelsiusFeeder = 0.0;

        public double velocityRotPerSecondLeft = 0.0;
        public double velocityRotPerSecondRight = 0.0;
        public double velocityRotPerSecondFeeder = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight,
            double velocityRotPerSecondFeeder) {
    }

    public default void stop() {

    }

    public default void stopShooter() {
    }

    public default void stopFeeder() {

    }
}
