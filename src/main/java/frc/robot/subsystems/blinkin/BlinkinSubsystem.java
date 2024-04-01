package frc.robot.subsystems.blinkin;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;

public class BlinkinSubsystem extends SubsystemBase {
    private final BlinkinIO io;
    private final BlinkinIOInputsAutoLogged inputs = new BlinkinIOInputsAutoLogged();

    private BlinkinState state = BlinkinState.DEFAULT;

    private final Timer blinkController = new Timer();

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
        blinkController.start();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Blinkin", inputs);

        if (blinkController.hasElapsed(state.pattern.blinkIntervalSeconds)) {
            blinkController.reset();
            if (inputs.color == state.color) {
                io.setColor(BlinkinColors.UNKNOWN);
            } else {
                io.setColor(state.color);
            }
        }

        Logger.recordOutput("Blinkin/ColorSetpoint", state.color);
        Logger.recordOutput("Blinkin/TimeUntilBlink", state.pattern.blinkIntervalSeconds - blinkController.get());
        Logger.recordOutput("Blinkin/Pattern/Blinks?", state.pattern.blinks);
        Logger.recordOutput("Blinkin/Pattern/IntervalSecond", state.pattern.blinkIntervalSeconds);
    }

    public void setState(BlinkinState state) {
        this.state = state;
    }

    public Command setStateCommand(BlinkinState state) {
        return new InstantCommand(() -> setState(state));
    }
}
