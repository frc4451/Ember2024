package frc.robot.subsystems.amptrap;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.AmpTrapConstants;

public class AmpTrapIOTalonFX implements AmpTrapIO {
    private final TalonFX roller = new TalonFX(AmpTrapConstants.kAmpTrapCanID);

    private final StatusSignal<Double> appliedVoltage = roller.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsius = roller.getDeviceTemp();
    private final StatusSignal<Double> currentAmperage = roller.getSupplyCurrent();
    private final StatusSignal<Double> velocityRotPerSecond = roller.getSupplyCurrent();

    private final VelocityVoltage velocity = new VelocityVoltage(0);

    public AmpTrapIOTalonFX() {
        this.roller.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                                .withDutyCycleClosedLoopRampPeriod(1.0)));

        velocity.Slot = 0;
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
    public void stop() {
        this.setVoltage(0.0);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.roller.set(percent);
    }
}
