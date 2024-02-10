package frc.robot.subsystems.amptrap;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AmptrapConstants;

public class AmptrapIOSparkMax implements AmptrapIO {
    private final CANSparkMax pivot = new CANSparkMax(AmptrapConstants.kPivotCanId, MotorType.kBrushless);

    private final RelativeEncoder encoder = pivot.getEncoder();

    public AmptrapIOSparkMax() {
        this.pivot.restoreFactoryDefaults();
        this.pivot.setIdleMode(IdleMode.kCoast);
        this.pivot.setClosedLoopRampRate(1.0);
        this.pivot.setInverted(true);
        this.pivot.burnFlash();

        this.encoder.setPositionConversionFactor(2.0 * Math.PI / AmptrapConstants.kPivotReduction);
        this.encoder.setPosition(0);
    }

    @Override
    public void updateInputs(AmptrapIOInputs inputs) {
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
    public void setPercentOutput(double decimalPercent) {
        this.pivot.set(decimalPercent);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.encoder.setPosition(angle.getRadians());
    }
}
