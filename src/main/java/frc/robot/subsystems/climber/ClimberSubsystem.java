package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final PIDController pidController = new PIDController(
            Constants.ClimberConstants.kClimberP,
            Constants.ClimberConstants.kClimberI,
            Constants.ClimberConstants.kClimberD);

    private double setpointRotations = 0.0;

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

        this.reset();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Climber", this.inputs);

        if (DriverStation.isDisabled()) {
            this.setSetpoint(0.0);
            this.io.stop();
            return;
        }

        this.runPID();
    }

    public void reset() {
        io.setPosition(0.0);
    }

    private void setSetpoint(double setpointRotations) {
        this.setpointRotations = setpointRotations;
        this.pidController.setSetpoint(setpointRotations);
    }

    public Command setSetpointCommand(double angle) {
        return new InstantCommand(() -> this.setSetpoint(angle), this);
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.setpointRotations), this);
    }

    public void runPID() {
        this.io.setVelocity(this.pidController.calculate(this.inputs.velocityRotPerSecond));
    }

    public Command pidCommand() {
        return new RunCommand(() -> {
            double output = this.pidController.calculate(this.io.getPosition());
            doThings(output);
        }, this);
    }

    public void doThings(double output) {
        this.io.setVoltage(MathUtil.clamp(output, -12.0, 12.0));
    }

    public Command runPercentCommand(DoubleSupplier decimalPercent) {
        return new RunCommand(() -> this.io.setPercentOutput(decimalPercent.getAsDouble()), this);
    }
}
