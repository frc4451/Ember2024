package frc.robot.reusable_io.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeambreakDigitalInput implements BeambreakIO {
    private final DigitalInput beamBreak;

    public BeambreakDigitalInput(int dioChannel) {
        this.beamBreak = new DigitalInput(dioChannel);
    }

    @Override
    public void updateInputs(BeambreakIOInputs inputs) {
        inputs.isConnected = true;
        inputs.isActivated = !beamBreak.get();
    }
}
