package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ClimberIOSim implements ClimberIO {
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController pidController = new PIDController(1, 0, 0);

    private double appliedVoltage = 0.0;
    private double velocityRotPerSecond = 0.0;
    private double positionRotations = 0.0;
    private boolean closedLoop = false;

    public void updateInputs(ClimberIOInputs inputs) {
        if (closedLoop) {
            appliedVoltage = 12.0
                    * pidController.calculate(sim.getAngularVelocityRPM() / 60.0, velocityRotPerSecond);
        }
        sim.update(0.02);
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmperage = sim.getCurrentDrawAmps();
        inputs.velocityRotPerSecond = sim.getAngularVelocityRPM() / 60.0;
        this.positionRotations += sim.getAngularVelocityRPM() / 60 * 0.02;
        inputs.positionRotations = positionRotations;
    }

    public void setVelocity(double velocityRotPerSecond) {
        closedLoop = true;
        this.velocityRotPerSecond = velocityRotPerSecond;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        appliedVoltage = voltage;
        sim.setInputVoltage(voltage);
    }

    @Override
    public void setPercentOutput(double decimalPercent) {
        setVoltage(12.0 * decimalPercent);
    }

    @Override
    public void setPosition(double position) {
        this.positionRotations = position;
    }
}
