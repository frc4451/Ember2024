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
import frc.robot.Constants.PhoenixConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private static final double kInchesPerRotation = ElevatorConstants.kElevatorSpoolDiameter
            * Math.PI
            / ElevatorConstants.kElevatorReduction;

    private final TalonFX io = new TalonFX(ElevatorConstants.kElevatorCanID);

    private final StatusSignal<Double> appliedVoltage = io.getMotorVoltage();
    private final StatusSignal<Double> velocityRotPerSec = io.getVelocity();
    private final StatusSignal<Double> currentAmperage = io.getSupplyCurrent();
    private final StatusSignal<Double> temperatureCelsius = io.getDeviceTemp();
    private final StatusSignal<Double> positionRotations = io.getPosition();

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

        StatusSignal.setUpdateFrequencyForAll(
                PhoenixConstants.defaultStatusSignalFrequencyHz,
                appliedVoltage,
                velocityRotPerSec,
                temperatureCelsius,
                currentAmperage,
                positionRotations);
        io.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        StatusSignal.refreshAll(
                appliedVoltage,
                velocityRotPerSec,
                temperatureCelsius,
                currentAmperage,
                positionRotations);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.velocityInchesPerSecond = velocityRotPerSec.getValueAsDouble() * kInchesPerRotation;
        inputs.currentAmperage = currentAmperage.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
        inputs.positionInches = positionRotations.getValueAsDouble() * kInchesPerRotation;
    }

    @Override
    public void setVoltage(double voltage) {
        this.io.setVoltage(voltage);
    }

    @Override
    public void setPercentOutput(double percentDecimal) {
        this.io.set(percentDecimal);
    }

    @Override
    public void setPosition(double positionInches) {
        this.io.setPosition(positionInches / kInchesPerRotation);
    }
}
