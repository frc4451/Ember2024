package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX top = new TalonFX(ShooterConstants.kTopShooterCanID); // leader
    private final TalonFX bottom = new TalonFX(ShooterConstants.kBottomShooterCanID); // follower

    private final StatusSignal<Double> topAmperage = top.getSupplyCurrent();
    private final StatusSignal<Double> topVoltage = top.getMotorVoltage();
    private final StatusSignal<Double> topTempCelsius = top.getDeviceTemp();
    private final StatusSignal<Double> topVelocity = top.getVelocity();
    private final StatusSignal<Double> bottomVoltage = bottom.getMotorVoltage();
    private final StatusSignal<Double> bottomAmperage = bottom.getSupplyCurrent();
    private final StatusSignal<Double> bottomTempCelsius = bottom.getDeviceTemp();
    private final StatusSignal<Double> bottomVelocity = bottom.getVelocity();

    private final VelocityVoltage velocity = new VelocityVoltage(0);

    public ShooterIOTalonFX() {
        for (TalonFX talon : new TalonFX[] { top, bottom }) {
            talon.getConfigurator().apply(
                    new TalonFXConfiguration()
                            .withSlot0(new Slot0Configs()
                                    .withKV(0)
                                    .withKP(0)
                                    .withKI(0)
                                    .withKD(0)));
        }

        bottom.setControl(new Follower(top.getDeviceID(), true));

        velocity.Slot = 0;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusSignal.refreshAll(
                topVoltage,
                topAmperage,
                topTempCelsius,
                topVelocity,
                bottomVoltage,
                bottomAmperage,
                bottomTempCelsius,
                bottomVelocity);
        inputs.appliedVoltage = new double[] {
                topVoltage.getValueAsDouble(),
                bottomVoltage.getValueAsDouble()
        };
        inputs.currentAmperage = new double[] {
                topAmperage.getValueAsDouble(),
                bottomAmperage.getValueAsDouble()
        };
        inputs.temperatureCelsius = new double[] {
                topTempCelsius.getValueAsDouble(),
                bottomTempCelsius.getValueAsDouble()
        };
        inputs.velocityRotPerSecond = new double[] {
                topVelocity.getValueAsDouble(),
                bottomVelocity.getValueAsDouble()
        };
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        top.setControl(velocity.withVelocity(velocityRotPerSecond));
    }
}
