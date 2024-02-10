package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX pivot = new TalonFX(IntakeConstants.kPivotCanId);
    private final double positionConversionFactor = 2.0 * Math.PI / IntakeConstants.kPivotReduction;

    public PivotIOFalcon() {
        this.pivot.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                                .withDutyCycleClosedLoopRampPeriod(1.0)));
        this.pivot.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVoltage = this.pivot.getDutyCycle().getValueAsDouble()
                * this.pivot.getMotorVoltage().getValueAsDouble();
        inputs.temperatureCelsius = this.pivot.getDeviceTemp().getValueAsDouble();
        inputs.currentAmperage = this.pivot.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        this.pivot.setVoltage(voltage);
    }

    @Override
    public void stop() {
        this.setVoltage(0.0);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.pivot.setPosition(angle.getRadians() / positionConversionFactor);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.pivot.set(percent);
    }
}
