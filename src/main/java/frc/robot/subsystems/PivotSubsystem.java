package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class PivotSubsystem extends SubsystemBase {
    // TODO make sure motor type is right
    private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.kPivotCanId, MotorType.kBrushless);

    private final RelativeEncoder encoder = pivot.getEncoder();

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    private final ArmFeedforward feedforward = new ArmFeedforward(
            0.0,
            0.0,
            0.0,
            0.0);

    private final PIDController feedback = new PIDController(
            0.0,
            0.0,
            0.0);

    private final PowerDistribution pdp;

    private Rotation2d setpoint = new Rotation2d();

    public PivotSubsystem(PowerDistribution pdp) {
        this.pdp = pdp;

        this.setAngle(Rotation2d.fromDegrees(184.0));

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
    }

    public Command getPivotCommand() {
        return new InstantCommand(this::pivot, this);
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

    private void pivot() {
        double feedforwardOut = this.feedforward.calculate(
                this.getAngle().getRadians(),
                IntakeConstants.kPivotVelocityRadiansPerSecond);

        double feedbackOut = this.feedback.calculate(
                this.getAngle().getRadians(),
                setpoint.getRadians());

        double velocity = feedforwardOut + feedbackOut;

        this.runAtPercent(velocity / pdp.getVoltage());
    }
}
