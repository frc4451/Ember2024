package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
    // private static double momentOfInertiaKgMSquared = 0.0000032998;
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController pidController = new PIDController(1, 0, 0);

    private double velocityRotPerSecond = 0.0;

    private double appliedVoltage = 0.0;

    private boolean closedLoopFeeder = false;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        if (closedLoopFeeder) {
            appliedVoltage = 12.0
                    * pidController.calculate(sim.getAngularVelocityRPM() / 60.0,
                            velocityRotPerSecond);
        }

        sim.setInputVoltage(appliedVoltage);

        sim.update(Constants.loopback);

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmperage = sim.getCurrentDrawAmps();
        inputs.velocityRotPerSecond = sim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        closedLoopFeeder = true;
        this.velocityRotPerSecond = velocityRotPerSecond;
    }

    @Override
    public void setVoltage(double voltage) {
        closedLoopFeeder = false;
        appliedVoltage = voltage;
    }
}
