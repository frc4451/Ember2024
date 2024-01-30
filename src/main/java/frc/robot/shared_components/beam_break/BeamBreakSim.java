package frc.robot.shared_components.beam_break;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class BeamBreakSim implements BeamBreakIO {
    public final DIOSim beamBreakSim;

    public BeamBreakSim(int dioChannel) {
        beamBreakSim = new DIOSim(dioChannel);
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.connected = beamBreakSim.getIsInput();
        inputs.value = beamBreakSim.getValue();
    }
}
