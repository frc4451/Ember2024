package frc.robot.reusable_io.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeambreakIO {
    @AutoLog
    public static class BeambreakIOInputs {
        public boolean isObstructed = false;
    }

    public default void updateInputs(BeambreakIOInputs inputs) {
    }

    public default void overrideObstructed(boolean isObstructed) {
    }
}
