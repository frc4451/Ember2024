package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final BeambreakIO beambreak;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

    private final PIDController pidController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD);

    private double setpointInches = 0.0;

    private final ElevatorVisualizer measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
    private final ElevatorVisualizer setpointVisualizer = new ElevatorVisualizer("Setpoint", Color.kGreen);

    public ElevatorSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ElevatorIOTalonFX(ElevatorConstants.kElevatorCanID, false);
                beambreak = new BeambreakDigitalInput(ElevatorConstants.kBeambreakChannel);
                break;
            case SIM:
                io = new ElevatorIOSim();
                beambreak = new BeambreakDigitalInput(ElevatorConstants.kBeambreakChannel);
                break;
            case REPLAY:
            default:
                io = new ElevatorIO() {
                };
                beambreak = new BeambreakIO() {
                };
                break;
        }

        this.pidController.setTolerance(0.05);
        this.reset();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        this.beambreak.updateInputs(this.beambreakInputs);

        Logger.processInputs("Elevator", this.inputs);
        Logger.processInputs("Elevator/BeamBreak", this.beambreakInputs);

        if (DriverStation.isDisabled()) {
            this.setSetpoint(0.0);
            this.io.stop();
        }

        Logger.recordOutput("Elevator/SetpointInches", setpointInches);

        // Log Mechanisms
        measuredVisualizer.update(this.inputs.positionInches);
        setpointVisualizer.update(this.setpointInches);

        BobotState.setElevatorUp(this.inputs.positionInches <= 1.0);
    }

    public void reset() {
        io.setPosition(0.0);
    }

    private void setSetpoint(double setpoint) {
        this.setpointInches = MathUtil.clamp(setpoint, ElevatorConstants.kMinHeightInches,
                ElevatorConstants.kMaxHeightInches);
        this.pidController.setSetpoint(this.setpointInches);
    }

    public Command setSetpointCommand(double positionInches) {
        return new InstantCommand(() -> this.setSetpoint(positionInches), this);
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.inputs.positionInches), this);
    }

    public Command pidCommand() {
        return new RunCommand(() -> {
            double output = this.pidController.calculate(this.inputs.positionInches);
            setVoltage(output);
        }, this);
    }

    public void setVoltage(double voltage) {
        this.io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    public Trigger elevatorIsDown() {
        return new Trigger(() -> this.inputs.positionInches <= 1.0);
    }

    public Trigger elevatorIsMoving() {
        return new Trigger(() -> this.inputs.velocityInchesPerSecond >= 1.0);
    }

    public Trigger elevatorIsUp() {
        return new Trigger(() -> this.inputs.positionInches >= 10.0);
    }
}
