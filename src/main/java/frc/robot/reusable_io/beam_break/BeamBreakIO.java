package frc.robot.reusable_io.beam_break;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    public static class BeamBreakIOInputs {
        public boolean isConnected = false;
        public boolean isActivated = false;
    }

    public default void updateInputs(BeamBreakIOInputs inputs) {
    }
}
