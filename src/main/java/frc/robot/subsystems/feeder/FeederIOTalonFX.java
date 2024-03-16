package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.PhoenixConstants;
import frc.robot.Constants.ShooterConstants;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX feeder = new TalonFX(ShooterConstants.kFeederCanID, PhoenixConstants.kCANivoreName);

    private final StatusSignal<Double> voltage = feeder.getMotorVoltage();
    private final StatusSignal<Double> amperage = feeder.getSupplyCurrent();
    private final StatusSignal<Double> tempCelsius = feeder.getDeviceTemp();
    private final StatusSignal<Double> velocity = feeder.getVelocity();

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    public FeederIOTalonFX() {
        feeder.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSlot0(new Slot0Configs()
                                .withKV(0.12)
                                .withKP(0.12)
                                .withKI(0.0)
                                .withKD(0.0)));
        velocityVoltage.Slot = 0;

        StatusSignal.setUpdateFrequencyForAll(
                PhoenixConstants.kStatusSignalFrequencyHz,
                voltage,
                amperage,
                tempCelsius,
                velocity);
        feeder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        StatusSignal.refreshAll(
                voltage,
                amperage,
                tempCelsius,
                velocity);

        inputs.appliedVoltage = voltage.getValueAsDouble();
        inputs.currentAmperage = amperage.getValueAsDouble();
        inputs.temperatureCelsius = tempCelsius.getValueAsDouble();
        inputs.velocityRotPerSecond = velocity.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        feeder.setControl(velocityVoltage.withVelocity(velocityRotPerSecond));
    }

    @Override
    public void setVoltage(double voltageFeeder) {
        feeder.setVoltage(voltageFeeder);
    }
}
