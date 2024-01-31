package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.reusable_io.beam_break.BeamBreakDigitalInput;
import frc.robot.reusable_io.beam_break.BeamBreakIO;
import frc.robot.reusable_io.beam_break.BeamBreakIOInputsAutoLogged;
import frc.robot.reusable_io.beam_break.BeamBreakSim;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO topIO;
    private final IntakeIO bottomIO;
    private final BeamBreakIO beamBreakIO;

    private final IntakeIOInputsAutoLogged topInputs = new IntakeIOInputsAutoLogged();
    private final IntakeIOInputsAutoLogged bottomInputs = new IntakeIOInputsAutoLogged();
    private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

    public IntakeSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                topIO = new IntakeIOTalonFX(1, true);
                bottomIO = new IntakeIOTalonFX(2, false);
                beamBreakIO = new BeamBreakDigitalInput(0);
                break;
            case SIM:
                topIO = new IntakeIOSim();
                bottomIO = new IntakeIOSim();
                beamBreakIO = new BeamBreakSim(0);
                break;
            case REPLAY:
            default:
                topIO = new IntakeIO() {
                };
                bottomIO = new IntakeIO() {
                };
                beamBreakIO = new BeamBreakIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {
        this.topIO.updateInputs(this.topInputs);
        this.bottomIO.updateInputs(this.bottomInputs);
        this.beamBreakIO.updateInputs(this.beamBreakInputs);

        Logger.processInputs("Intake/Top", this.topInputs);
        Logger.processInputs("Intake/Bottom", this.topInputs);
        Logger.processInputs("Intake/BeamBreak", this.beamBreakInputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.topIO.setVelocity(0.0);
            this.bottomIO.setVelocity(0.0);
        }
    }

    public Command setVelocityCommand(double topSpeed, double bottomSpeed) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> this.topIO.setVelocity(topSpeed)),
                new InstantCommand(() -> this.bottomIO.setVelocity(bottomSpeed)));
    }

    public Command stopCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> this.topIO.setVelocity(0.0)),
                new InstantCommand(() -> this.bottomIO.setVelocity(0.0)));
    }

    public Trigger beamBreakIsNotCovered() {
        return new Trigger(() -> this.beamBreakInputs.isActivated);
    }
}
