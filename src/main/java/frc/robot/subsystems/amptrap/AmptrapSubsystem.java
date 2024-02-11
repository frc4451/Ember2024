package frc.robot.subsystems.amptrap;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;

public class AmptrapSubsystem extends SubsystemBase {
    private final AmptrapIO io;
    private final AmptrapIOInputsAutoLogged inputs = new AmptrapIOInputsAutoLogged();

    private Rotation2d angle = new Rotation2d();
    private Rotation2d setpoint = new Rotation2d();

    private final PIDController pidController = new PIDController(
            Constants.IntakeConstants.kPivotP,
            Constants.IntakeConstants.kPivotI,
            Constants.IntakeConstants.kPivotD);

    public AmptrapSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new AmptrapIOTalonFX();
                break;
            case SIM:
                io = new AmptrapIOSim();
                break;
            case REPLAY:
            default:
                io = new AmptrapIO() {
                };
                break;
        }

        setAngle(AmptrapLocation.INITIAL.angle);
        setSetpoint(AmptrapLocation.INITIAL.angle);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Amptrap", this.inputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.setSetpoint(AmptrapLocation.INITIAL.angle);
            this.io.stop();
        }

        this.angle = new Rotation2d(this.inputs.relativeAngleRad);

        Logger.recordOutput("Amptrap/Angle", getAngle().getDegrees());
        Logger.recordOutput("Amptrap/SetpointAngle", getSetpoint().getDegrees());
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
        this.pidController.setSetpoint(angle.getDegrees());
    }

    public Command setSetpointCommand(Rotation2d angle) {
        return new InstantCommand(() -> this.setSetpoint(angle), this);
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.angle), this);
    }

    public Command amptrapPIDCommand() {
        return new RunCommand(() -> {
            double output = this.pidController.calculate(this.getAngle().getDegrees());
            useOutput(output);
        }, this);
    }

    public void useOutput(double output) {
        this.io.setVoltage(MathUtil.clamp(output, -12, 12));
    }

    public Command runPercentCommand(DoubleSupplier decimalPercent) {
        return new RunCommand(() -> this.io.setPercentOutput(decimalPercent.getAsDouble()), this);
    }
}
