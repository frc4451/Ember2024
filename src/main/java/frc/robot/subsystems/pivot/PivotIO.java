package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double appliedVoltageLeader = 0.0;
        public double currentAmperageLeader = 0.0;
        public double temperatureCelsiusLeader = 0.0;
        public double relativeAngleRadLeader = 0.0;

        public double appliedVoltageFollower = 0.0;
        public double currentAmperageFollower = 0.0;
        public double temperatureCelsiusFollower = 0.0;
        public double relativeAngleRadFollower = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {
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
