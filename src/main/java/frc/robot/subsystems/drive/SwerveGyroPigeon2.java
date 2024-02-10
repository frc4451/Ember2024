package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class SwerveGyroPigeon2 implements SwerveGyroIO {
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.kGyroCanId);

    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> yawRate = pigeon.getAngularVelocityZWorld();

    public SwerveGyroPigeon2() {
        Pigeon2Configurator configurator = pigeon.getConfigurator();
        configurator.apply(new Pigeon2Configuration()); // reset default settings
        configurator.setYaw(0); // reset yaw on initialization (is this neccessary?)
    }

    public void updateInputs(SwerveGyroIOInputs inputs) {
        inputs.isConnected = StatusSignal.refreshAll(yaw, yawRate).isOK();
        inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawRate.getValueAsDouble());
    }

    public void zero() {
        pigeon.reset();
    }
}
