package frc.robot.subsystems.amptrap;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.AmpTrapConstants;
import frc.robot.Constants.PhoenixConstants;

public class AmpTrapIOTalonFX implements AmpTrapIO {
    private final TalonFX roller = new TalonFX(AmpTrapConstants.kCanId);

    private final StatusSignal<Double> appliedVoltage = roller.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsius = roller.getDeviceTemp();
    private final StatusSignal<Double> currentAmperage = roller.getSupplyCurrent();
    private final StatusSignal<Double> velocityRotPerSecond = roller.getVelocity();

    private final VelocityVoltage velocity = new VelocityVoltage(0);

    public AmpTrapIOTalonFX(boolean isInverted) {
        this.roller.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Coast).withInverted(
                                        isInverted
                                                ? InvertedValue.Clockwise_Positive
                                                : InvertedValue.CounterClockwise_Positive))
                        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                                .withDutyCycleClosedLoopRampPeriod(1.0))
                        .withSlot0(new Slot0Configs()
                                .withKV(0.12)
                                .withKP(0.1)
                                .withKI(0)
                                .withKD(0)));

        velocity.Slot = 0;

        StatusSignal.setUpdateFrequencyForAll(
                PhoenixConstants.kStatusSignalFrequencyHz,
                appliedVoltage,
                temperatureCelsius,
                currentAmperage,
                velocityRotPerSecond);
        roller.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(AmpTrapIOInputs inputs) {
        StatusSignal.refreshAll(appliedVoltage, temperatureCelsius, currentAmperage, velocityRotPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
        inputs.currentAmperage = currentAmperage.getValueAsDouble();
        inputs.velocityRotPerSecond = velocityRotPerSecond.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        roller.setControl(velocity.withVelocity(velocityRotPerSecond));
    }

    @Override
    public void setVoltage(double voltage) {
        this.roller.setVoltage(voltage);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.roller.set(percent);
    }
}
