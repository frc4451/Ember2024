package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmperage = 0.0;
        public double temperatureCelsius = 0.0;

        public double velocityInchesPerSecond = 0.0;
        public double positionInches = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setPercentOutput(double percentDecimal) {
    }

    public default void stop() {
        setVoltage(0.0);
    }

    public default void setPosition(double positionInches) {
    }
}
