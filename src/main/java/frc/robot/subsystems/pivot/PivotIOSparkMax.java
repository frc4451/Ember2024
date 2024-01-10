package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;

public class PivotIOSparkMax implements PivotIO {
    private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.kPivotCanId, MotorType.kBrushless);

    private final RelativeEncoder encoder = pivot.getEncoder();

    public PivotIOSparkMax() {
        this.encoder.setPositionConversionFactor(2.0 * Math.PI / IntakeConstants.kPivotReduction);

        this.pivot.restoreFactoryDefaults();
        this.pivot.setIdleMode(IdleMode.kBrake);
        this.pivot.setClosedLoopRampRate(1.0);
        this.pivot.burnFlash();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVoltage = this.pivot.getAppliedOutput() * this.pivot.getBusVoltage();
        inputs.temperatureCelsius = this.pivot.getMotorTemperature();
        inputs.currentAmperage = this.pivot.getOutputCurrent();

        inputs.relativeAngleRad = this.encoder.getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        this.pivot.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.encoder.setPosition(angle.getRadians());
    }
}
