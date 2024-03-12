package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PhoenixConstants;

public class PivotIOTalonFX implements PivotIO {
    private static final double kRadiansPerRotation = 2.0 * Math.PI / IntakeConstants.kPivotReduction;

    private final TalonFX pivotLeader = new TalonFX(IntakeConstants.kPivotLeaderCanId);
    private final TalonFX pivotFollower = new TalonFX(IntakeConstants.kPivotFollowerCanId);

    private final StatusSignal<Double> appliedVoltageLeader = pivotLeader.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsiusLeader = pivotLeader.getDeviceTemp();
    private final StatusSignal<Double> currentAmperageLeader = pivotLeader.getSupplyCurrent();
    private final StatusSignal<Double> positionRotationsLeader = pivotLeader.getPosition();

    private final StatusSignal<Double> appliedVoltageFollower = pivotFollower.getMotorVoltage();
    private final StatusSignal<Double> temperatureCelsiusFollower = pivotFollower.getDeviceTemp();
    private final StatusSignal<Double> currentAmperageFollower = pivotFollower.getSupplyCurrent();
    private final StatusSignal<Double> positionRotationsFollower = pivotFollower.getPosition();

    public PivotIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(1.0));
        this.pivotLeader.getConfigurator().apply(config);
        this.pivotFollower.getConfigurator().apply(config);

        this.pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), false));

        StatusSignal.setUpdateFrequencyForAll(
                PhoenixConstants.kStatusSignalFrequencyHz,
                appliedVoltageLeader,
                temperatureCelsiusLeader,
                currentAmperageLeader,
                positionRotationsLeader,
                appliedVoltageFollower,
                temperatureCelsiusFollower,
                currentAmperageFollower,
                positionRotationsFollower);
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
                appliedVoltageFollower,
                temperatureCelsiusFollower,
                currentAmperageFollower,
                positionRotationsFollower);
        inputs.appliedVoltageLeader = appliedVoltageLeader.getValueAsDouble();
        inputs.temperatureCelsiusLeader = temperatureCelsiusLeader.getValueAsDouble();
        inputs.currentAmperageLeader = currentAmperageLeader.getValueAsDouble();
        inputs.positionRadLeader = positionRotationsLeader.getValueAsDouble() * kRadiansPerRotation;

        inputs.appliedVoltageFollower = appliedVoltageFollower.getValueAsDouble();
        inputs.temperatureCelsiusFollower = temperatureCelsiusFollower.getValueAsDouble();
        inputs.currentAmperageFollower = currentAmperageFollower.getValueAsDouble();
        inputs.positionRadFollower = positionRotationsFollower.getValueAsDouble() * kRadiansPerRotation;
    }

    @Override
    public void setVoltage(double voltage) {
        this.pivotLeader.setVoltage(voltage);
    }

    @Override
    public void stop() {
        this.setVoltage(0.0);
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
}
