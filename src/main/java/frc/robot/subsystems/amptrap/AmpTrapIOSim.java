package frc.robot.subsystems.amptrap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class AmpTrapIOSim implements AmpTrapIO {
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim rollerSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController rollerPidController = new PIDController(1, 0, 0);

    private double appliedVoltage = 0.0;
    private double velocityRotPerSecond = 0.0;

    @Override
    public void updateInputs(AmpTrapIOInputs inputs) {
        appliedVoltage = 12.0
                * rollerPidController.calculate(rollerSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecond);

        rollerSim.setInputVoltage(appliedVoltage);

        rollerSim.update(0.02); // 20 ms is the standard periodic loop time

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmperage = rollerSim.getCurrentDrawAmps();
        inputs.velocityRotPerSecond = rollerSim.getAngularVelocityRPM() / 60.0;

    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        this.velocityRotPerSecond = velocityRotPerSecond;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        appliedVoltage = voltage;
    }

    @Override
    public void setPercentOutput(double decimalPercent) {
        setVoltage(12.0 * decimalPercent);
    }
}
