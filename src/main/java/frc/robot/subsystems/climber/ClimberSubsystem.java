package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0);

    private final TrapezoidProfile motionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    ClimberConstants.kMaxVelocityInchesPerSecond,
                    ClimberConstants.kMaxAccelerationInchesPerSecondSquared));

    private double setpointInches = 0.0;

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

        // this.pidController.setTolerance(0.05);
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

        Logger.recordOutput("Climber/SetpointInches", setpointInches);
    }

    public void setVoltage(double voltage) {
        this.io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    public void reset() {
        io.setPosition(0.0);
    }

    private void setSetpoint(double setpoint) {
        this.setpointInches = clampSetpoint(setpoint);
    }

    public Command setSetpointCommand(double positionInches) {
        return new InstantCommand(() -> this.setSetpoint(positionInches));
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.inputs.positionInches), this);
    }

    private double clampSetpoint(double setpointInches) {
        return MathUtil.clamp(
                setpointInches,
                ClimberConstants.kMinHeightInches,
                ClimberConstants.kMaxHeightInches);
    }

    public Command pidCommand() {
        return new RunCommand(() -> {
            TrapezoidProfile.State output = motionProfile.calculate(
                    0.02,
                    new TrapezoidProfile.State(this.inputs.positionInches, this.inputs.velocityInchesPerSecond),
                    new TrapezoidProfile.State(clampSetpoint(this.setpointInches), 0));
            this.io.runSetpoint(output.position, ff.calculate(output.position, output.velocity));
        }, this);
    }

    public Command runPercentCommand(DoubleSupplier decimalPercent) {
        return new RunCommand(
                () -> setSetpoint(clampSetpoint(decimalPercent.getAsDouble() * ClimberConstants.kMaxHeightInches)));
    }
}
