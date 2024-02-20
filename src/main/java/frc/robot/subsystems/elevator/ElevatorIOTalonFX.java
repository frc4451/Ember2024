package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private static final double kPositionConversionFactor = 1.0 / ElevatorConstants.kElevatorReduction
            * ElevatorConstants.kElevatorSpoolDiameter * Math.PI;
    private final TalonFX io = new TalonFX(ElevatorConstants.kElevatorCanID);

    private final StatusSignal<Double> appliedVoltage = io.getMotorVoltage();
    private final StatusSignal<Double> velocityInchesPerSecond = io.getVelocity();
    private final StatusSignal<Double> currentAmperage = io.getSupplyCurrent();
    private final StatusSignal<Double> temperatureCelsius = io.getDeviceTemp();
    private final StatusSignal<Double> positionInches = io.getPosition();

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    public ElevatorIOTalonFX(int deviceId, boolean isInverted) {
        this.io.getConfigurator()
                .apply(new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(
                                        isInverted
                                                ? InvertedValue.Clockwise_Positive
                                                : InvertedValue.CounterClockwise_Positive))
                        .withSlot0(new Slot0Configs()
                                .withKV(0.12)
                                .withKP(0.1)
                                .withKI(0)
                                .withKD(0)));
        velocityVoltage.Slot = 0;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        StatusSignal.refreshAll(
                appliedVoltage,
                velocityInchesPerSecond,
                temperatureCelsius,
                currentAmperage,
                positionInches);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.velocityInchesPerSecond = velocityInchesPerSecond.getValueAsDouble() * kPositionConversionFactor;
        inputs.currentAmperage = currentAmperage.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
        inputs.positionInches = positionInches.getValueAsDouble() * kPositionConversionFactor;
    }

    @Override
    public void setVoltage(double voltage) {
        this.io.setVoltage(voltage);
    }

    @Override
    public void setPosition(double position) {
        this.io.setPosition(position / kPositionConversionFactor);
    }
}
