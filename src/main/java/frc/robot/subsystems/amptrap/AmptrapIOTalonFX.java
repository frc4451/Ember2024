package frc.robot.subsystems.amptrap;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;

public class AmptrapIOTalonFX implements AmptrapIO {
    private static final double kPositionConversionFactor = 2.0 * Math.PI / IntakeConstants.kPivotReduction;

    private final TalonFX pivot = new TalonFX(IntakeConstants.kPivotCanId);

    private final StatusSignal<Double> appliedVoltage = pivot.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsius = pivot.getDeviceTemp();
    private final StatusSignal<Double> currentAmperage = pivot.getSupplyCurrent();

    public AmptrapIOTalonFX() {
        this.pivot.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                                .withDutyCycleClosedLoopRampPeriod(1.0)));
    }

    @Override
    public void updateInputs(AmptrapIOInputs inputs) {
        StatusSignal.refreshAll(appliedVoltage, temperatureCelsius, currentAmperage);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
        inputs.currentAmperage = currentAmperage.getValueAsDouble();
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
        this.pivot.setPosition(angle.getRadians() / kPositionConversionFactor);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.pivot.set(percent);
    }
}
