package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Velocity;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX top = new TalonFX(-1);
    private final TalonFX bottom = new TalonFX(-2);

    private final StatusSignal<Double> topVoltage = top.getSupplyVoltage();
    private final StatusSignal<Double> bottomVoltage = bottom.getSupplyVoltage();
    private final StatusSignal<Double> topVelocity = bottom.getVelocity();
    private final StatusSignal<Double> bottomVelocity = bottom.getVelocity();

    private final VelocityVoltage velocity = new VelocityVoltage(0);

    public IntakeIOTalonFX() {
        this.top.getConfigurator().apply(new TalonFXConfiguration());
        this.top.setNeutralMode(NeutralModeValue.Brake);
        this.bottom.getConfigurator().apply(new TalonFXConfiguration());
        this.bottom.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        StatusSignal.refreshAll(topVelocity, topVoltage, bottomVelocity, bottomVoltage);
        inputs.appliedVoltage = new double[] {
                topVoltage.getValueAsDouble(),
                bottomVoltage.getValueAsDouble()
        };
        inputs.velocityRotPerSecond = new double[] {
                topVelocity.getValueAsDouble(),
                bottomVelocity.getValueAsDouble()
        };
    }

    @Override
    public void setVelocity(double velocityRotPerSecondtop, double velocityRotPerSecondbottom) {
        top.setControl(velocity.withVelocity(velocityRotPerSecondtop));
        bottom.setControl(velocity.withVelocity(velocityRotPerSecondbottom));
    }

    @Override
    public void setFree() {
        top.setControl(new CoastOut());
        bottom.setControl(new CoastOut());
    }
}
