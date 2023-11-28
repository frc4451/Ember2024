package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.IntakeConstants;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private Rotation2d angle = new Rotation2d();
    private Rotation2d setpoint = new Rotation2d();

    public PivotSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new PivotIOSparkMax();
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

        setAngle(PivotLocation.INITIAL.angle);
        setSetpoint(PivotLocation.INITIAL.angle);
    }

    @Override
    public void periodic() {
        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.io.stop();
        }

        this.io.updateInputs(this.inputs);
        Logger.getInstance().processInputs("Intake/Pivot", this.inputs);

        this.angle = new Rotation2d(this.inputs.relativeAngleRad);
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

    public void setSetpoint(Rotation2d angle) {
        this.setpoint = angle;
    }

    public Command setSetpointCommand(Rotation2d angle) {
        return new RunCommand(() -> this.setSetpoint(angle));
    }

    public void runAtPercent(double percent) {
        double angleDegrees = this.getAngle().getDegrees();
        if ((angleDegrees < IntakeConstants.kPivotMinDegrees && percent < 0.0)
                || (angleDegrees > IntakeConstants.kPivotMaxDegrees && percent > 0.0)) {
            this.io.stop();
        } else {
            this.io.setVoltage(12.0 * percent / 2.0);
        }
    }

    public void pivot() {
        double velocity = 0.6 * this.getSetpoint().minus(this.getAngle()).getDegrees();
        this.io.setVoltage(velocity);
    }

    public Command pivotCommand() {
        return new InstantCommand(this::pivot, this);
    }
}
