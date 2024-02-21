package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmperage = 0.0;
        public double temperatureCelsius = 0.0;

        public double velocityInchesPerSecond = 0.0;
        public double positionInches = 0.0;

        public double velocityRadPerSecond = 0.0;
        public double positionRad = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setPercentOutput(double percentDecimal) {
    }

    public default void setPosition(double positionInches) {
    }

    public default void stop() {
        setVoltage(0.0);
    }
}
