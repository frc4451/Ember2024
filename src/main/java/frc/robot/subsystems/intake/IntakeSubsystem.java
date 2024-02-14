package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.reusable_io.beambreak.BeambreakIOSim;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final BeambreakIO beambreak;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

    public IntakeSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new IntakeIOTalonFX(1, false);
                beambreak = new BeambreakDigitalInput(IntakeConstants.kBeamBreakChannel);
                break;
            case SIM:
                io = new IntakeIOSim();
                beambreak = new BeambreakIOSim(IntakeConstants.kBeamBreakChannel);
                break;
            case REPLAY:
            default:
                io = new IntakeIO() {
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

        Logger.processInputs("Intake/Motor", this.inputs);
        Logger.processInputs("Intake/BeamBreak", this.beambreakInputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.io.setVelocity(0.0);
        }
    }

    public Command setVelocityCommand(double velocityRotPerSecond) {
        return new InstantCommand(() -> this.io.setVelocity(velocityRotPerSecond), this);
    }

    public Command stopCommand() {
        return setVelocityCommand(0);
    }

    // For testing and sim
    public Command setBeamBreakActivatedCommand(boolean value) {
        return new InstantCommand(() -> {
            this.beambreak.overrideActivated(value);
        });
    }

    // For testing and sim
    public Command toggleBeamBrakeActivatedCommand() {
        return new InstantCommand(() -> {
            this.beambreak.overrideActivated(!this.beambreakIsActivated().getAsBoolean());
        });
    }

    public Trigger beambreakIsActivated() {
        return new Trigger(() -> this.beambreakInputs.isActivated);
    }
}
