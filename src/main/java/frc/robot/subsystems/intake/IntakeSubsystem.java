package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.reusable_io.beambreak.BeambreakSim;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO top;
    private final IntakeIO bottom;
    private final BeambreakIO beambreak;

    private final IntakeIOInputsAutoLogged topInputs = new IntakeIOInputsAutoLogged();
    private final IntakeIOInputsAutoLogged bottomInputs = new IntakeIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

    public IntakeSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                top = new IntakeIOTalonFX(1, true);
                bottom = new IntakeIOTalonFX(2, false);
                beambreak = new BeambreakDigitalInput(IntakeConstants.kBeamBreakChannel);
                break;
            case SIM:
                top = new IntakeSim();
                bottom = new IntakeSim();
                beambreak = new BeambreakSim(IntakeConstants.kBeamBreakChannel);
                break;
            case REPLAY:
            default:
                top = new IntakeIO() {
                };
                bottom = new IntakeIO() {
                };
                beambreak = new BeambreakIO() {
                };

                break;
        }
    }

    @Override
    public void periodic() {
        this.top.updateInputs(this.topInputs);
        this.bottom.updateInputs(this.bottomInputs);
        this.beambreak.updateInputs(this.beambreakInputs);

        Logger.processInputs("Intake/Top", this.topInputs);
        Logger.processInputs("Intake/Bottom", this.topInputs);
        Logger.processInputs("Intake/BeamBreak", this.beambreakInputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.top.setVelocity(0.0);
            this.bottom.setVelocity(0.0);
        }
    }

    public Command setVelocityCommand(double topSpeed, double bottomSpeed) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> this.top.setVelocity(topSpeed)),
                new InstantCommand(() -> this.bottom.setVelocity(bottomSpeed)));
    }

    public Command stopCommand() {
        return setVelocityCommand(0, 0);
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
