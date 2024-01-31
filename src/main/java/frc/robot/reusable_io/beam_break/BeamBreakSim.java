package frc.robot.reusable_io.beam_break;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class BeamBreakSim implements BeamBreakIO {
    public final DIOSim beamBreakSim;

    public BeamBreakSim(int dioChannel) {
        beamBreakSim = new DIOSim(dioChannel);
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.isConnected = beamBreakSim.getInitialized();
        inputs.isActivated = !beamBreakSim.getValue();
    }
}
