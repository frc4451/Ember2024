package frc.robot.shared_components.beam_break;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakAdafruit implements BeamBreakIO {
    private final DigitalInput beamBreak;

    public BeamBreakAdafruit(int dioChannel) {
        this.beamBreak = new DigitalInput(dioChannel);
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.connected = true;
        inputs.value = beamBreak.get();
    }
}
