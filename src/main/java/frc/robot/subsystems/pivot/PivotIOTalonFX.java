package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PhoenixConstants;
import frc.robot.Constants.PivotConstants;

public class PivotIOTalonFX implements PivotIO {
    private static final double kRadiansPerRotation = 2.0 * Math.PI / PivotConstants.kPivotReduction;

    private final TalonFX pivotLeader = new TalonFX(PivotConstants.kPivotLeaderCanId, PhoenixConstants.kCANivoreName);
    private final TalonFX pivotFollower = new TalonFX(PivotConstants.kPivotFollowerCanId,
            PhoenixConstants.kCANivoreName);

    private final StatusSignal<Double> appliedVoltageLeader = pivotLeader.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsiusLeader = pivotLeader.getDeviceTemp();
    private final StatusSignal<Double> currentAmperageLeader = pivotLeader.getSupplyCurrent();
    private final StatusSignal<Double> positionRotationsLeader = pivotLeader.getPosition();
    private final StatusSignal<Double> velocityRotPerSecLeader = pivotLeader.getVelocity();

    private final StatusSignal<Double> appliedVoltageFollower = pivotFollower.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsiusFollower = pivotFollower.getDeviceTemp();
    private final StatusSignal<Double> currentAmperageFollower = pivotFollower.getSupplyCurrent();
    private final StatusSignal<Double> positionRotationsFollower = pivotFollower.getPosition();
    private final StatusSignal<Double> velocityRotPerSecFollower = pivotFollower.getVelocity();

    private final PositionVoltage position = new PositionVoltage(0);

    public PivotIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(1.0))
                .withSlot0(new Slot0Configs()
                        .withKV(0.12)
                        .withKP(4.0)
                        .withKI(0)
                        .withKD(0));

        position.Slot = 0;

        this.pivotLeader.getConfigurator().apply(config);
        this.pivotFollower.getConfigurator().apply(config);

        this.pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), false));

        StatusSignal.setUpdateFrequencyForAll(
                PhoenixConstants.kStatusSignalFrequencyHz,
                appliedVoltageLeader,
                temperatureCelsiusLeader,
                currentAmperageLeader,
                positionRotationsLeader,
                velocityRotPerSecLeader,
                appliedVoltageFollower,
                temperatureCelsiusFollower,
                currentAmperageFollower,
                positionRotationsFollower,
                velocityRotPerSecLeader);
        this.pivotFollower.optimizeBusUtilization();
        this.pivotLeader.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        StatusSignal.refreshAll(
                appliedVoltageLeader,
                temperatureCelsiusLeader,
                currentAmperageLeader,
                positionRotationsLeader,
                velocityRotPerSecLeader,
                appliedVoltageFollower,
                temperatureCelsiusFollower,
                currentAmperageFollower,
                positionRotationsFollower,
                velocityRotPerSecFollower);
        inputs.appliedVoltageLeader = appliedVoltageLeader.getValueAsDouble();
        inputs.temperatureCelsiusLeader = temperatureCelsiusLeader.getValueAsDouble();
        inputs.currentAmperageLeader = currentAmperageLeader.getValueAsDouble();
        inputs.positionRadLeader = positionRotationsLeader.getValueAsDouble() * kRadiansPerRotation;
        inputs.velocityRadPerSecLeader = velocityRotPerSecLeader.getValueAsDouble() * kRadiansPerRotation;

        inputs.appliedVoltageFollower = appliedVoltageFollower.getValueAsDouble();
        inputs.temperatureCelsiusFollower = temperatureCelsiusFollower.getValueAsDouble();
        inputs.currentAmperageFollower = currentAmperageFollower.getValueAsDouble();
        inputs.positionRadFollower = positionRotationsFollower.getValueAsDouble() * kRadiansPerRotation;
        inputs.velocityRadPerSecFollower = velocityRotPerSecFollower.getValueAsDouble() * kRadiansPerRotation;
    }

    @Override
    public void setVoltage(double voltage) {
        this.pivotLeader.setVoltage(voltage);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.pivotLeader.setPosition(angle.getRadians() / kRadiansPerRotation);
        this.pivotFollower.setPosition(angle.getRadians() / kRadiansPerRotation);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.pivotLeader.set(percent);
    }

    @Override
    public void setPosition(double positionRad) {
        this.pivotLeader.setControl(position.withPosition(positionRad / kRadiansPerRotation));
    }
}
