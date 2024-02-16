package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVoltage = 0.0;
        public double velocityRotPerSecond = 0.0;
        public double currentAmperage = 0.0;
        public double temperatureCelsius = 0.0;
        public double positionRotations = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setVelocity(double velocityRotPerSecond) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setPercentOutput(double decimalPercent) {

    }

    public default void stop() {
        setVoltage(0.0);
    }

    public default void setPosition(double position) {
    }

    public default double getPosition() {
        return -0.0;
    }
}
