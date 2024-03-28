package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

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
import frc.utils.GarageUtils;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

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
                break;
            case SIM:
                io = new ElevatorIOSim();
                break;
            case REPLAY:
            default:
                io = new ElevatorIO() {
                };
                break;
        }

        this.pidController.setTolerance(0.05);
        this.reset();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);

        Logger.processInputs("Elevator", this.inputs);

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
        return new Trigger(() -> MathUtil.isNear(
                ElevatorConstants.kMinHeightInches,
                this.inputs.positionInches,
                1.0));
    }

    public Trigger elevatorIsAtSubwooferShot() {
        return new Trigger(() -> MathUtil.isNear(
                ElevatorConstants.kSubwooferShotHeightInches,
                this.inputs.positionInches,
                2.0));
    }

    public Trigger elevatorIsAtAmp() {
        return new Trigger(() -> MathUtil.isNear(
                ElevatorConstants.kAmpScoreHeightInches,
                this.inputs.positionInches,
                1.0));
    }

    public Trigger elevatorIsAtTrap() {
        return new Trigger(() -> MathUtil.isNear(
                ElevatorConstants.kTrapScoreHeightInches,
                this.inputs.positionInches,
                1.0));
    }

    public Trigger elevatorIsUp() {
        return new Trigger(() -> MathUtil.isNear(
                ElevatorConstants.kPivotClearanceHeightInches,
                this.inputs.positionInches,
                1.0));
    }

    public void runPercentOutput(double percentDecimal) {
        double output = GarageUtils.percentWithSoftStops(
                percentDecimal,
                this.inputs.positionInches + this.inputs.velocityInchesPerSecond,
                ElevatorConstants.kMinHeightInches,
                ElevatorConstants.kMaxHeightInches);
        this.io.setPercentOutput(output);
    }

    public Command runPercentOutputCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> this.runPercentOutput(percentDecimal.getAsDouble()), this);
    }
}
