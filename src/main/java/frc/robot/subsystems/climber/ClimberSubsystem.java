package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    private TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    ClimberConstants.profileConstraints.maxVelocity,
                    ClimberConstants.profileConstraints.maxAcceleration));

    private TrapezoidProfile.State minState = new TrapezoidProfile.State(
            Units.inchesToMeters(ClimberConstants.kMinHeightInches),
            0);
    private TrapezoidProfile.State maxState = new TrapezoidProfile.State(
            Units.inchesToMeters(ClimberConstants.kMaxHeightInches),
            0);

    private TrapezoidProfile.State nextState = new TrapezoidProfile.State();
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();

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

        Logger.recordOutput("Climber/SetpointInches", setpointInches);

        Logger.recordOutput("Climber/GoalStatePosition", this.goalState.position);
        Logger.recordOutput("Climber/GoalStateVelocity", this.goalState.velocity);
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

    public Command pidCommand() {
        return new RunCommand(() -> {
            double output = this.pidController.calculate(this.inputs.positionInches);
            setVoltage(output);
        }, this);
    }

    public Command runPercentCommand(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> this.io.setPercentOutput(
                GarageUtils.percentSmoothBetweenValues(
                        percentDecimal.getAsDouble(),
                        this.inputs.positionInches,
                        ClimberConstants.kMinHeightInches,
                        ClimberConstants.kMaxHeightInches)),
                this);
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
    public Command runTrapezoidProfile(DoubleSupplier percentDecimal) {
        return new RunCommand(() -> {
            // Determine which direction we need the motors to spin
            double direction = Math.signum(percentDecimal.getAsDouble());

            // Determine which state we want to end up
            this.goalState = direction == 1 ? maxState : minState;

            // Calculate the next step in the profile control loop
            this.nextState = this.profile.calculate(
                    Constants.loopback,
                    new TrapezoidProfile.State(
                            this.inputs.positionInches * direction,
                            this.inputs.velocityInchesPerSecond * percentDecimal.getAsDouble()),
                    this.goalState);

            // Provide the TrapezoidProfile State to PID Controller to keep position.
            this.setSetpoint(Units.metersToInches(this.nextState.position));
        });
    }
}
