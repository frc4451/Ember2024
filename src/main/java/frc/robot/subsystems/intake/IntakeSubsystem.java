package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
                io = new IntakeIOTalonFX(IntakeConstants.kIntakeCanId, false);
                beambreak = new BeambreakDigitalInput(IntakeConstants.kBeambreakChannel);
                break;
            case SIM:
                io = new IntakeIOSim();
                beambreak = new BeambreakIOSim(IntakeConstants.kBeambreakChannel);
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
        this.io.updateInputs(this.inputs);
        this.beambreak.updateInputs(this.beambreakInputs);

        Logger.processInputs("Intake", this.inputs);
        Logger.processInputs("Intake/Beambreak", this.beambreakInputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.io.stop();
        }
    }

    public Command setVelocityCommand(double velocityRotPerSecond) {
        return new InstantCommand(() -> this.io.setVelocity(velocityRotPerSecond), this);
    }

    public Command setVelocityThenStopCommand(double velocityRotPerSecond) {
        return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this).finallyDo(io::stop);
    }

    public Command setVelocityBeambreakCommand(double velocityRotPerSecond) {
        return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this)
                .unless(beambreakIsObstructed())
                .until(beambreakIsObstructed())
                .andThen(stopCommand());
    }

    public Command stopCommand() {
        return new InstantCommand(this.io::stop, this);
    }

    // For testing and sim
    public Command setBeambreakObstructedCommand(boolean value) {
        return new InstantCommand(() -> {
            this.beambreak.overrideObstructed(value);
        });
    }

    // For testing and sim
    public Command toggleBeambreakObstructedCommand() {
        return new InstantCommand(() -> {
            this.beambreak.overrideObstructed(!this.beambreakInputs.isObstructed);
        });
    }

    public Trigger beambreakIsObstructed() {
        return new Trigger(() -> this.beambreakInputs.isObstructed);
    }
}
