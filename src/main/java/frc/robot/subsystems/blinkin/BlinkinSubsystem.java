package frc.robot.subsystems.blinkin;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;

public class BlinkinSubsystem extends SubsystemBase {
    private final BlinkinIO io;
    private final BlinkinIOInputsAutoLogged inputs = new BlinkinIOInputsAutoLogged();

    public BlinkinSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new BlinkinIOSpark();
                break;
            case SIM:
                io = new BlinkinIOSim();
                break;
            case REPLAY:
            default:
                io = new BlinkinIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Blinkin", inputs);
    }

    public void setColor(BlinkinColors color) {
        io.setColor(color);
    }
}
