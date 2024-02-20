package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;

public class PivotIOSparkMax implements PivotIO {
    private final CANSparkMax pivotLeader = new CANSparkMax(IntakeConstants.kPivotLeaderCanId, MotorType.kBrushless);
    private final CANSparkMax pivotFollower = new CANSparkMax(IntakeConstants.kPivotFollowerCanId,
            MotorType.kBrushless);

    private final RelativeEncoder leaderEncoder = pivotLeader.getEncoder();
    private final RelativeEncoder followerEncoder = pivotFollower.getEncoder();

    public PivotIOSparkMax() {
        this.pivotLeader.restoreFactoryDefaults();
        this.pivotLeader.setIdleMode(IdleMode.kCoast);
        this.pivotLeader.setClosedLoopRampRate(1.0);
        this.pivotLeader.setInverted(true);
        this.pivotLeader.burnFlash();

        this.leaderEncoder.setPositionConversionFactor(2.0 * Math.PI / IntakeConstants.kPivotReduction);
        this.followerEncoder.setPositionConversionFactor(2.0 * Math.PI / IntakeConstants.kPivotReduction);
        this.leaderEncoder.setPosition(0);
        this.followerEncoder.setPosition(0);

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVoltageLeader = this.pivotLeader.getAppliedOutput() * this.pivotLeader.getBusVoltage();
        inputs.temperatureCelsiusLeader = this.pivotLeader.getMotorTemperature();
        inputs.currentAmperageLeader = this.pivotLeader.getOutputCurrent();
        inputs.positionRadLeader = this.leaderEncoder.getPosition();

        inputs.appliedVoltageLeader = this.pivotFollower.getAppliedOutput() * this.pivotLeader.getBusVoltage();
        inputs.temperatureCelsiusLeader = this.pivotFollower.getMotorTemperature();
        inputs.currentAmperageLeader = this.pivotFollower.getOutputCurrent();
        inputs.positionRadLeader = this.followerEncoder.getPosition();

    }

    @Override
    public void setVoltage(double voltage) {
        this.pivotLeader.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    @Override
    public void setPercentOutput(double percentDecimal) {
        this.pivotLeader.set(percentDecimal);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.leaderEncoder.setPosition(angle.getRadians());
    }
}
