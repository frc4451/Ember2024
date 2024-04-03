package frc.robot.subsystems.amptrap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.AmpTrapConstants;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.reusable_io.beambreak.BeambreakIOSim;

public class AmpTrapSubsystem extends SubsystemBase {
    private final AmpTrapIO io;
    private final BeambreakIO beambreak;

    private final AmpTrapIOInputsAutoLogged inputs = new AmpTrapIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

    public AmpTrapSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new AmpTrapIOTalonFX(true);
                beambreak = new BeambreakDigitalInput(AmpTrapConstants.kBeambreakChannel);
                break;
            case SIM:
                io = new AmpTrapIOSim();
                beambreak = new BeambreakIOSim(AmpTrapConstants.kBeambreakChannel);
                break;
            case REPLAY:
            default:
                io = new AmpTrapIO() {
                };
                beambreak = new BeambreakIO() {
                };
                break;
        }

    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        this.beambreak.updateInputs(this.beambreakInputs);

        Logger.processInputs("AmpTrap", this.inputs);
        Logger.processInputs("AmpTrap/Beambreak", this.beambreakInputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.io.stop();
        }
    }

    public void runPercent(double percent) {
        io.setPercentOutput(percent);
    }

    public Command runPercentCommand(double percent) {
        return new RunCommand(() -> runPercent(percent), this);
    }

    public void setVelocity(double velocityRotPerSecond) {
        io.setVelocity(velocityRotPerSecond);
    }

    public Command setVelocityCommand(double velocityRotPerSecond) {
        return new InstantCommand(() -> setVelocity(velocityRotPerSecond), this);
    }

    public Command stopCommand() {
        return new InstantCommand(io::stop, this);
    }

    /** This is specifically for sim testing, as beambreaks are not simulated */
    public Command overrideBeambreakObstructedCommand(boolean value) {
        return new InstantCommand(() -> {
            this.beambreak.overrideObstructed(value);
        });
    }

    public Trigger beambreakIsObtructed() {
        return new Trigger(() -> this.beambreakInputs.isObstructed);
    }
}
