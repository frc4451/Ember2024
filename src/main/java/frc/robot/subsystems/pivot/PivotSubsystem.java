package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.bobot_state.BobotState;
import frc.utils.GarageUtils;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private Rotation2d angle = new Rotation2d();

    /**
     * Setpoint in degrees
     */
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /**
     * Goal in degrees
     */
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    Units.radiansToDegrees(PivotConstants.kMaxVelocityRadiansPerSecond),
                    Units.radiansToDegrees(PivotConstants.kMaxAccelerationRadiansPerSecondSquared)));

    // private final TrapezoidProfile.State minState = new TrapezoidProfile.State(
    // PivotLocation.kSoftMin.angle.getRadians(),
    // 0.0);

    // private final TrapezoidProfile.State maxState = new TrapezoidProfile.State(
    // PivotLocation.kElevatorUpSoftMax.angle.getRadians(),
    // 0.0);

    // Mechanisms
    private final PivotVisualizer measuredVisualizer = new PivotVisualizer("Measured", Color.kBlack);
    private final PivotVisualizer setpointVisualizer = new PivotVisualizer("Setpoint", Color.kYellow);
    private final PivotVisualizer goalVisualizer = new PivotVisualizer("Goal", Color.kBlue);

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

        this.setAngle(PivotLocation.INITIAL.angle);
        this.setSetpoint(PivotLocation.INITIAL.angle);
        this.setGoal(PivotLocation.INITIAL.angle);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Pivot", this.inputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.setGoal(PivotLocation.INITIAL.angle);
            this.io.stop();
        }

        this.angle = new Rotation2d(this.inputs.positionRadLeader);

        Logger.recordOutput("Pivot/Angle", angle.getDegrees());
        Logger.recordOutput("Pivot/SetpointAngle", setpoint.position);
        Logger.recordOutput("Pivot/GoalAngle", goal.position);

        Logger.recordOutput("Pivot/IsBelowElevatorThreshold", this.isBelowElevatorConflictTresholdBoolean());

        // Log Mechanisms - This needs to be recorded in Radians
        measuredVisualizer.update(angle.getRadians());
        setpointVisualizer.update(Units.degreesToRadians(setpoint.position));
        goalVisualizer.update(Units.degreesToRadians(goal.position));
    }

    public void setAngle(Rotation2d angle) {
        this.io.setAngle(angle);
    }

    public Rotation2d getAngle() {
        return this.angle;
    }

    public TrapezoidProfile.State getGoal() {
        return this.goal;
    }

    private void setSetpoint(Rotation2d angle) {
        this.setpoint = new TrapezoidProfile.State(angle.getDegrees(), 0.0);
    }

    private void setGoal(Rotation2d angle) {
        this.goal = new TrapezoidProfile.State(angle.getDegrees(), 0.0);
    }

    public Command setGoalCommand(Rotation2d angle) {
        return new InstantCommand(() -> this.setGoal(angle));
    }

    private void setEverythingCurrent() {
        setSetpoint(getAngle());
        setGoal(getAngle());
    }

    public Command setEverythingCurrentCommand() {
        return new InstantCommand(this::setEverythingCurrent);
    }

    private void runTrapezoidProfile() {
        setpoint = trapezoidProfile.calculate(
                Constants.loopback,
                setpoint,
                goal);
        io.setPosition(Units.degreesToRadians(setpoint.position));
    }

    public Command runTrapezoidProfileCommand() {
        return new RunCommand(this::runTrapezoidProfile, this);
    }

    public double getUpperLimit() {
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
    public Command controlOutOfTheElevatorsWay() {
        return new RunCommand(() -> {
            setGoal(
                    Rotation2d.fromDegrees(MathUtil.clamp(
                            this.angle.getDegrees(),
                            PivotLocation.INITIAL.angle.getDegrees(),
                            PivotLocation.kElevatorDownSoftMax.angle.getDegrees())));
        });
    }

    public Command setGoalToAmpScoringPosition() {
        return new InstantCommand(() -> {
            setGoal(PivotLocation.kAmpScoringPosition.angle);
        });
    }

    public Command setGoalToTrapScoringPosition() {
        return new InstantCommand(() -> {
            setGoal(PivotLocation.kTrapScoringPosition.angle);
        });
    }

    public Command runPercentCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> {
            double output = GarageUtils.percentWithSoftStops(
                    percentDecimal.getAsDouble(),
                    getAngle().getDegrees(),
                    PivotLocation.kSoftMin.angle.getDegrees(),
                    getUpperLimit());
            io.setPercentOutput(output);
        }, this);
    }

    public Command controlGoalToSpeakerCommand() {
        return new RunCommand(() -> {
            setGoal(Rotation2d.fromDegrees(BobotState.getShootingCalculation().angleDegrees()));
        });
    }

    /**
     * The Pivot cannot exceed 42degrees when the elevator is down.
     */
    public Trigger isBelowElevatorConflictTreshold() {
        return new Trigger(this::isBelowElevatorConflictTresholdBoolean);
    }

    /**
     * The Pivot cannot exceed 42degrees when the elevator is down.
     */
    private boolean isBelowElevatorConflictTresholdBoolean() {
        return this.getAngle().getDegrees() <= PivotLocation.kElevatorDownHardMax.angle.getDegrees();
    }

    public Trigger isNearAmpScoringAngle() {
        return new Trigger(() -> MathUtil.isNear(
                PivotLocation.kAmpScoringPosition.angle.getDegrees(),
                this.getAngle().getDegrees(),
                1.0));
    }

    public Trigger isNearTrapScoringAngle() {
        return new Trigger(() -> MathUtil.isNear(
                PivotLocation.kTrapScoringPosition.angle.getDegrees(),
                this.getAngle().getDegrees(),
                1.0));
    }

    public Trigger isNearBottom() {
        return new Trigger(() -> MathUtil.isNear(
                PivotLocation.INITIAL.angle.getDegrees(),
                this.getAngle().getDegrees(),
                1.0));
    }

    public Trigger isNearGoal() {
        return new Trigger(() -> MathUtil.isNear(
                getGoal().position,
                getAngle().getDegrees(),
                1.0));
    }
}
