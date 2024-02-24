package frc.robot.reusable_io.beambreak;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class BeambreakIOSim implements BeambreakIO {
    public final DIOSim beambreakSim;

    public BeambreakIOSim(int dioChannel) {
        beambreakSim = new DIOSim(dioChannel);
    }

    @Override
    public void updateInputs(BeambreakIOInputs inputs) {
        inputs.isConnected = beambreakSim.getInitialized();
        inputs.isObstructed = !beambreakSim.getValue();
    }

    @Override
    public void overrideObstructed(boolean isObstructed) {
        this.beambreakSim.setValue(!isObstructed);
    }
}
