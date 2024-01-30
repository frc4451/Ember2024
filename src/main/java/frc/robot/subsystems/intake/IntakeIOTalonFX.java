package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX talon;
    private final StatusSignal<Double> voltage;
    private final StatusSignal<Double> talonVelocity;
    private final VelocityVoltage velocityVoltage;

    public IntakeIOTalonFX(int deviceId, boolean isInverted) {
        talon = new TalonFX(deviceId);
        voltage = talon.getSupplyVoltage();
        talonVelocity = talon.getVelocity();
        velocityVoltage = new VelocityVoltage(0);

        this.talon.getConfigurator()
                .apply(new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSlot0(new Slot0Configs().withKV(0.1).withKP(0.1).withKI(0).withKD(0)));
        this.talon.setNeutralMode(NeutralModeValue.Brake);
        this.talon.setInverted(isInverted);
        velocityVoltage.Slot = 0;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        StatusSignal.refreshAll(talonVelocity, voltage);
        inputs.appliedVoltage = voltage.getValueAsDouble();
        inputs.velocityRotPerSecond = talonVelocity.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        talon.setControl(velocityVoltage.withVelocity(velocityRotPerSecond));
    }

    @Override
    public void setVoltage(double voltage) {
        talon.setVoltage(voltage);
    }
}
