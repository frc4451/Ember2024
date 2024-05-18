package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PhoenixConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX talon;
    private final StatusSignal<Double> voltage;
    private final StatusSignal<Double> dutyCycle;
    private final StatusSignal<Double> velocity;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public IntakeIOTalonFX(int deviceId, boolean isInverted) {
        talon = new TalonFX(deviceId, PhoenixConstants.kCANivoreName);
        voltage = talon.getMotorVoltage();
        dutyCycle = talon.getDutyCycle();
        velocity = talon.getVelocity();

        talon.getConfigurator()
                .apply(new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(
                                        isInverted
                                                ? InvertedValue.Clockwise_Positive
                                                : InvertedValue.CounterClockwise_Positive))
                        .withSlot0(new Slot0Configs()
                                .withKV(0.12)
                                .withKP(1)
                                .withKI(0)
                                .withKD(0)));
        velocityVoltage.Slot = 0;

        StatusSignal.setUpdateFrequencyForAll(PhoenixConstants.kStatusSignalFrequencyHz, voltage, dutyCycle, velocity);
        talon.optimizeBusUtilization();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        StatusSignal.refreshAll(velocity, dutyCycle, voltage);
        inputs.appliedVoltage = voltage.getValueAsDouble();
        inputs.appliedDutyCycle = dutyCycle.getValueAsDouble();
        inputs.velocityRotPerSecond = velocity.getValueAsDouble() / IntakeConstants.kReduction;
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        talon.setControl(velocityVoltage.withVelocity(velocityRotPerSecond * IntakeConstants.kReduction));
    }

    @Override
    public void setPercentOutput(double percentDecimal) {
        talon.setControl(dutyCycleOut.withOutput(percentDecimal));
    }

    @Override
    public void setVoltage(double voltage) {
        talon.setVoltage(voltage);
    }
}
