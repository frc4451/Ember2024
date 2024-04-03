package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.PhoenixConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX left = new TalonFX(ShooterConstants.kLeftShooterCanID, PhoenixConstants.kCANivoreName);
    private final TalonFX right = new TalonFX(ShooterConstants.kRightShooterCanID, PhoenixConstants.kCANivoreName);

    private final StatusSignal<Double> leftVoltage = left.getMotorVoltage();
    private final StatusSignal<Double> rightVoltage = right.getMotorVoltage();

    private final StatusSignal<Double> leftAmperage = left.getSupplyCurrent();
    private final StatusSignal<Double> rightAmperage = right.getSupplyCurrent();

    private final StatusSignal<Double> leftTempCelsius = left.getDeviceTemp();
    private final StatusSignal<Double> rightTempCelsius = right.getDeviceTemp();

    private final StatusSignal<Double> leftVelocity = left.getVelocity();
    private final StatusSignal<Double> rightVelocity = right.getVelocity();

    private final VelocityVoltage velocity = new VelocityVoltage(0);

    public ShooterIOTalonFX() {
        left.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSlot0(new Slot0Configs()
                                .withKV(0.12)
                                .withKP(0.12)
                                .withKI(0.0)
                                .withKD(0.0)));
        right.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive))
                        .withSlot0(new Slot0Configs()
                                .withKV(0.12)
                                .withKP(0.12)
                                .withKI(0.0)
                                .withKD(0.0)));
        velocity.Slot = 0;

        StatusSignal.setUpdateFrequencyForAll(
                PhoenixConstants.kStatusSignalFrequencyHz,
                leftVoltage,
                rightVoltage,
                leftAmperage,
                rightAmperage,
                leftTempCelsius,
                rightTempCelsius,
                leftVelocity,
                rightVelocity);
        left.optimizeBusUtilization();
        right.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusSignal.refreshAll(
                leftVoltage,
                rightVoltage,
                leftAmperage,
                rightAmperage,
                leftTempCelsius,
                rightTempCelsius,
                leftVelocity,
                rightVelocity);

        inputs.appliedVoltageLeft = leftVoltage.getValueAsDouble();
        inputs.appliedVoltageRight = rightVoltage.getValueAsDouble();

        inputs.currentAmperageLeft = leftAmperage.getValueAsDouble();
        inputs.currentAmperageRight = rightAmperage.getValueAsDouble();

        inputs.temperatureCelsiusLeft = leftTempCelsius.getValueAsDouble();
        inputs.temperatureCelsiusRight = rightTempCelsius.getValueAsDouble();

        inputs.velocityRotPerSecondLeft = leftVelocity.getValueAsDouble();
        inputs.velocityRotPerSecondRight = rightVelocity.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        left.setControl(velocity.withVelocity(velocityRotPerSecondLeft));
        right.setControl(velocity.withVelocity(velocityRotPerSecondRight));
    }

    @Override
    public void setVoltage(double voltageLeft, double voltageRight) {
        left.setVoltage(voltageLeft);
        right.setVoltage(voltageRight);
    }
}
