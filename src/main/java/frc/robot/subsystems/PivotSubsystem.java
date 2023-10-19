package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class PivotSubsystem extends SubsystemBase {
    private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.kPivotCanId, MotorType.kBrushless);

    private final RelativeEncoder encoder = pivot.getEncoder();

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    // private final ArmFeedforward feedforward = new ArmFeedforward(
    // 0.0,
    // 0.0,
    // 0.0,
    // 0.0);

    // private final PIDController feedback = new PIDController(
    // 0.00,
    // 0.0,
    // 0.0);

    // private final PowerDistribution pdp;

    private Rotation2d setpoint = new Rotation2d();

    public PivotSubsystem() {
        // this.pdp = pdp;

        this.setAngle(PivotLocation.INITIAL.angle);
        this.setSetpoint(PivotLocation.INITIAL.angle);

        this.pivot.restoreFactoryDefaults();
        this.pivot.setIdleMode(IdleMode.kBrake);
        this.pivot.setClosedLoopRampRate(1.0);
        this.pivot.burnFlash();
    }

    public void setAngle(Rotation2d angle) {
        this.encoder.setPosition(angle.getDegrees() / 360.0 * 240.0);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(this.encoder.getPosition() / 240.0);
    }

    public Rotation2d getSetpoint() {
        return this.setpoint;
    }

    public void setSetpoint(Rotation2d angle) {
        this.setpoint = angle;
        // this.feedback.setSetpoint(angle.getRadians());
    }

    public Command setSetpointCommand(Rotation2d angle) {
        return new RunCommand(() -> this.setSetpoint(angle));
    }

    public void runAtPercent(double percent) {
        double angleDegrees = this.getAngle().getDegrees();
        if ((angleDegrees < IntakeConstants.kPivotMinDegrees && percent < 0.0)
                || (angleDegrees > IntakeConstants.kPivotMaxDegrees && percent > 0.0)) {
            this.pivot.set(0.0);
        } else {
            this.pivot.set(percent);
        }
    }

    public void pivot() {
        // double feedforwardOut = this.feedforward.calculate(
        // this.getSetpoint().getRadians(),
        // IntakeConstants.kPivotVelocityRadiansPerSecond);

        // double feedbackOut = this.feedback.calculate(
        // this.getAngle().getRadians());

        // double velocity = feedforwardOut + feedbackOut;

        double algebrafeedback = 0.05 * (this.getSetpoint().getDegrees() - this.getAngle().getDegrees());
        double velocity = algebrafeedback;

        // this.runAtPercent(Math.min(Math.max(velocity / pdp.getVoltage(), -0.5),
        // 0.5));
        // if Math.abs(velocity) < 0.1
        this.runAtPercent(Math.min(Math.max(velocity, -0.8), 0.8));
    }

    public Command pivotCommand() {
        return new InstantCommand(this::pivot, this);
    }
}
