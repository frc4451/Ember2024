package frc.robot.shared_components.beam_break;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    public static class BeamBreakIOInputs {
        public boolean connected = false;
        public boolean value = false;
    }

    public default void updateInputs(BeamBreakIOInputs inputs) {
    }
}
