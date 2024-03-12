package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.bobot_state.BobotState;
import frc.utils.GarageUtils;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private Rotation2d angle = new Rotation2d();
    private Rotation2d setpoint = new Rotation2d();

    private final PIDController pidController = new PIDController(
            IntakeConstants.kPivotP,
            IntakeConstants.kPivotI,
            IntakeConstants.kPivotD);

    // Mechanisms
    private final PivotVisualizer measuredVisualizer = new PivotVisualizer("Measured", Color.kBlack);
    private final PivotVisualizer setpointVisualizer = new PivotVisualizer("Setpoint", Color.kGreen);

    public PivotSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new PivotIOTalonFX();
                break;
            case SIM:
                io = new PivotIOSim();
                break;
            case REPLAY:
            default:
                io = new PivotIO() {
                };
                break;
        }

        this.pidController.setTolerance(Units.degreesToRadians(0.1));
        this.setAngle(PivotLocation.INITIAL.angle);
        this.setSetpoint(PivotLocation.INITIAL.angle);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Pivot", this.inputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.setSetpoint(PivotLocation.INITIAL.angle);
            this.io.stop();
        }

        this.angle = new Rotation2d(this.inputs.positionRadLeader);

        Logger.recordOutput("Pivot/Angle", getAngle().getDegrees());
        Logger.recordOutput("Pivot/SetpointAngle", getSetpoint().getDegrees());

        Logger.recordOutput("Pivot/IsBelowElevatorThreshold", this.pivotIsBelowElevatorMax().getAsBoolean());

        // Log Mechanisms - This needs to be recorded in Radians
        measuredVisualizer.update(getAngle().getRadians());
        setpointVisualizer.update(getSetpoint().getRadians());
    }

    public void setAngle(Rotation2d angle) {
        this.io.setAngle(angle);
    }

    public Rotation2d getAngle() {
        return this.angle;
    }

    public Rotation2d getSetpoint() {
        return this.setpoint;
    }

    private void setSetpoint(Rotation2d angle) {
        this.setpoint = angle;
        this.pidController.setSetpoint(setpoint.getDegrees());
    }

    public Command setSetpointCommand(Rotation2d angle) {
        return new InstantCommand(() -> this.setSetpoint(angle));
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.angle));
    }

    private void pid() {
        double output = this.pidController.calculate(this.getAngle().getDegrees());
        setVoltage(output);
    }

    public Command pidCommand() {
        return new RunCommand(this::pid, this);
    }

    public void setVoltage(double voltage) {
        this.io.setVoltage(MathUtil.clamp(voltage, -6.0, 6.0));
    }

    public double getPivotUpperLimit() {
        return BobotState.isElevatorUp()
                ? PivotLocation.kElevatorDownSoftMax.angle.getDegrees()
                : PivotLocation.kElevatorUpSoftMax.angle.getDegrees();
    }

    /**
     * Forces the pivot out of the way of the Elevator.
     *
     * We're using empirically gathered angles, but long term we should consider
     * a linear interpolation table for this.
     */
    public Command movePivotOutOfTheElevatorsWay() {
        return new RunCommand(() -> {
            setSetpoint(
                    Rotation2d.fromDegrees(MathUtil.clamp(
                            this.angle.getDegrees(),
                            PivotLocation.INITIAL.angle.getDegrees(),
                            PivotLocation.kElevatorDownSoftMax.angle.getDegrees())));
            pid();
        }, this);
    }

    public Command movePivotToAmpScoringPosition() {
        return new RunCommand(() -> {
            setSetpoint(PivotLocation.kAmpScoringPosition.angle);
            pid();
        }, this);
    }

    public Command movePivotToTrapScoringPosition() {
        return new RunCommand(() -> {
            setSetpoint(PivotLocation.kTrapScoringPosition.angle);
            pid();
        }, this);
    }

    public Command runPercentCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> {
            double output = GarageUtils.percentWithSoftStops(
                    percentDecimal.getAsDouble(),
                    getAngle().getDegrees(),
                    PivotLocation.kSoftMin.angle.getDegrees(),
                    getPivotUpperLimit());
            io.setPercentOutput(output);
        }, this);
    }

    public Command pivotToSpeakerCommand() {
        return new RunCommand(() -> {
            setSetpoint(Rotation2d.fromDegrees(BobotState.getShootingCalculation().angleDegrees()));
            pid();
        }, this);
    }

    public Command controlSetpointToSpeakerCommand() {
        return new RunCommand(() -> {
            setSetpoint(Rotation2d.fromDegrees(BobotState.getShootingCalculation().angleDegrees()));
        });
    }

    /**
     * The Pivot cannot exceed 42degrees when the elevator is down.
     */
    public Trigger pivotIsBelowElevatorMax() {
        return new Trigger(() -> this.getAngle().getDegrees() <= PivotLocation.kElevatorDownHardMax.angle.getDegrees());
    }

    public Trigger pivotIsNearAmpScoringAngle() {
        return new Trigger(() -> MathUtil.isNear(
                PivotLocation.kAmpScoringPosition.angle.getDegrees(),
                this.getAngle().getDegrees(),
                1.0));
    }

    public Trigger pivotIsNearTrapScoringAngle() {
        return new Trigger(() -> MathUtil.isNear(
                PivotLocation.kTrapScoringPosition.angle.getDegrees(),
                this.getAngle().getDegrees(),
                1.0));
    }

    public Trigger pivotIsNearBottom() {
        return new Trigger(() -> MathUtil.isNear(
                PivotLocation.INITIAL.angle.getDegrees(),
                this.getAngle().getDegrees(),
                1.0));
    }
}
