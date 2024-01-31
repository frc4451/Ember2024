package frc.robot.reusable_io.beam_break;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakDigitalInput implements BeamBreakIO {
    private final DigitalInput beamBreak;

    public BeamBreakDigitalInput(int dioChannel) {
        this.beamBreak = new DigitalInput(dioChannel);
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.isConnected = true;
        inputs.isActivated = !beamBreak.get();
    }
}
