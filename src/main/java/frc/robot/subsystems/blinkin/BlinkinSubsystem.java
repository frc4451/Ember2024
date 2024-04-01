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

    private BlinkinColors colorSetpoint = BlinkinColors.UNKNOWN;
    private BlinkinPattern pattern = BlinkinPattern.SOLID;

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

        if (blinkController.hasElapsed(pattern.blinkIntervalSeconds)) {
            blinkController.reset();
            if (inputs.color == colorSetpoint) {
                io.setColor(BlinkinColors.UNKNOWN);
            } else {
                io.setColor(colorSetpoint);
            }
        }

        Logger.recordOutput("Blinkin/ColorSetpoint", colorSetpoint);
        Logger.recordOutput("Blinkin/Blink/TimeUntilBlink", pattern.blinkIntervalSeconds - blinkController.get());
        Logger.recordOutput("Blinkin/Pattern/Blinks", pattern.blinks);
        Logger.recordOutput("Blinkin/Pattern/BlinkIntervalSecond", pattern.blinkIntervalSeconds);
    }

    public void setColor(BlinkinColors color) {
        colorSetpoint = color;
        io.setColor(color);
    }

    public void setPattern(BlinkinPattern pattern) {
        this.pattern = pattern;
        blinkController.reset();
        io.setColor(colorSetpoint);
    }

    public Command setColorCommand(BlinkinColors color) {
        return new InstantCommand(() -> setColor(color));
    }
}
