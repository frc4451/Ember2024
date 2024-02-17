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
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ClimberConstants;
import frc.utils.GarageUtils;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final PIDController pidController = new PIDController(
            ClimberConstants.kClimberP,
            ClimberConstants.kClimberI,
            ClimberConstants.kClimberD);

    private double setpointRot = 0.0;

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
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Climber", this.inputs);

        if (DriverStation.isDisabled()) {
            this.setSetpoint(0.0);
            this.io.stop();
        }

        Logger.recordOutput("Climber/SetpointRot", setpointRot);
    }

    public void reset() {
        io.setPosition(0.0);
    }

    private void setSetpoint(double setpoint) {
        this.setpointRot = MathUtil.clamp(setpoint, 0.0, ClimberConstants.kMaxHeightRotations);
        this.pidController.setSetpoint(setpoint);
    }

    public Command setSetpointCommand(double positionRot) {
        return new InstantCommand(() -> this.setSetpoint(positionRot), this);
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.inputs.positionRot), this);
    }

    public Command pidCommand() {
        return new RunCommand(() -> {
            double output = this.pidController.calculate(this.inputs.positionRot);
            setVoltage(output);
        }, this);
    }

    public void setVoltage(double voltage) {
        this.io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    public Command runPercentCommand(DoubleSupplier decimalPercent) {
        return new RunCommand(() -> this.io.setPercentOutput(
                GarageUtils.percentSmoothBetweenValues(
                        decimalPercent.getAsDouble(),
                        this.inputs.positionRot,
                        ClimberConstants.kMinHeightRotations,
                        ClimberConstants.kMaxHeightRotations)),
                this);
    }
}
