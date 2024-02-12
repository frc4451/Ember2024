package frc.robot.subsystems.amptrap;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AmpTrapIO {
    @AutoLog
    public static class AmpTrapIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmperage = 0.0;
        public double temperatureCelsius = 0.0;
        public double relativeAngleRad = 0.0;
    }

    public default void updateInputs(AmpTrapIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setPercentOutput(double decimalPercent) {
    }

    public default void stop() {
        setVoltage(0.0);
    }

    public default void setAngle(Rotation2d angle) {
    }
}
