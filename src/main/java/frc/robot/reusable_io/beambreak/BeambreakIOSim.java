package frc.robot.reusable_io.beambreak;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class BeambreakIOSim implements BeambreakIO {
    public final DIOSim beamBreakSim;

    public BeambreakIOSim(int dioChannel) {
        beamBreakSim = new DIOSim(dioChannel);
    }

    @Override
    public void updateInputs(BeambreakIOInputs inputs) {
        inputs.isConnected = beamBreakSim.getInitialized();
        inputs.isActivated = !beamBreakSim.getValue();
    }

    @Override
    public void overrideActivated(boolean isActivated) {
        this.beamBreakSim.setValue(!isActivated);
    }
}
