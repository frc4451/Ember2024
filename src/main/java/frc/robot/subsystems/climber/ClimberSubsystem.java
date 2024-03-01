package frc.robot.subsystems.climber;

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
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ClimberConstants;
import frc.utils.GarageUtils;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final PIDController pidController = new PIDController(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD);

    private double setpointInches = 0.0;

    private final ClimberVisualizer measuredVisualizer = new ClimberVisualizer("Measured", Color.kBlack);
    private final ClimberVisualizer setpointVisualizer = new ClimberVisualizer("Setpoint", Color.kGreen);

    public ClimberSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ClimberIOTalonFX(ClimberConstants.kClimberCanId, false);
                break;
            case SIM:
                io = new ClimberIOSim();
                break;
            case REPLAY:
            default:
                io = new ClimberIO() {
                };
                break;
        }

        this.pidController.setTolerance(0.05);
        this.reset();
        this.setSetpoint(0.0);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Climber", this.inputs);

        if (DriverStation.isDisabled()) {
            this.setSetpoint(0.0);
            this.io.stop();
        }

        Logger.recordOutput("Climber/SetpointInches", setpointInches);

        measuredVisualizer.update(this.inputs.positionInches);
        setpointVisualizer.update(setpointInches);
    }

    public void setVoltage(double voltage) {
        this.io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    public void reset() {
        io.setPosition(0.0);
    }

    private void setSetpoint(double setpointInches) {
        this.setpointInches = MathUtil.clamp(
                setpointInches,
                ClimberConstants.kMinHeightInches,
                ClimberConstants.kMaxHeightInches);
        this.pidController.setSetpoint(this.setpointInches);
    }

    public Command setSetpointCommand(double positionInches) {
        return new InstantCommand(() -> this.setSetpoint(positionInches), this);
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.inputs.positionInches), this);
    }

    public void pid() {
        double output = this.pidController.calculate(this.inputs.positionInches);
        setVoltage(output);
    }

    public Command pidCommand() {
        return new RunCommand(this::pid, this);
    }

    /**
     * Runs the Climber in a TrapezoidProfile control loop to keep user input
     * between both the higher and lower bounds of our Subsystem. We feed
     * the output position of our Trapezoid State to our PID Controller to
     * keep a steady voltage on our motors to hold position.
     *
     * @param percentDecimal - Joystick input for variable control.
     * @return TrapezoidProfiled Open Loop Command
     */
    public void runSetpointController(double percentDecimal) {
        // Determine which direction we need the motors to spin
        double direction = Math.signum(percentDecimal);

        // Determine which state we want to end up
        double setpoint = direction == 1
                ? ClimberConstants.kMaxHeightInches
                : ClimberConstants.kMinHeightInches;

        this.setSetpoint(setpoint);
    }

    /**
     * Runs the Climber in a TrapezoidProfile control loop to keep user input
     * between both the higher and lower bounds of our Subsystem. We feed
     * the output position of our Trapezoid State to our PID Controller to
     * keep a steady voltage on our motors to hold position.
     *
     * @param percentDecimal - Joystick input for variable control.
     * @return TrapezoidProfiled Open Loop Command
     */
    public Command runSetpointControllerCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> this.runSetpointController(percentDecimal.getAsDouble()));
    }

    public void runPercentOutput(double percentDecimal) {
        double output = GarageUtils.percentWithSoftStops(
                percentDecimal,
                this.inputs.positionInches,
                ClimberConstants.kMinHeightInches,
                ClimberConstants.kMaxHeightInches);
        this.io.setPercentOutput(output);
    }

    public Command runPercentOutputCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> this.runPercentOutput(percentDecimal.getAsDouble()), this);
    }

    public Command runClimberControlCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> {
            double input = percentDecimal.getAsDouble();
            if (input > 0.0) {
                runSetpointController(input);
                pid();
            } else {
                runPercentOutput(input);
            }
        }, this);
    }

}
