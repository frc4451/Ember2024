package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    // private static double momentOfInertiaKgMSquared = 0.0000032998;
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim wheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController pidController = new PIDController(1, 0, 0);

    private double velocityRotPerSecond = 0.0;

    private boolean closedLoop = false;
    private double appliedVoltage = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (closedLoop) {
            appliedVoltage = 12.0
                    * pidController.calculate(wheelSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecond);
        }

        wheelSim.setInputVoltage(appliedVoltage);
        wheelSim.update(0.02); // 20 ms is the standard periodic loop time

        inputs.appliedVoltage = appliedVoltage;
        inputs.appliedDutyCycle = appliedVoltage / 12.0;
        inputs.currentAmperage = wheelSim.getCurrentDrawAmps();
        inputs.velocityRotPerSecond = wheelSim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public void setVelocity(double velocityRotPerSecond) {
        closedLoop = true;
        this.velocityRotPerSecond = velocityRotPerSecond;
    }

    @Override
    public void setPercentOutput(double percentDecimal) {
        setVelocity(12 * percentDecimal);
    }

    @Override
    public void setVoltage(double voltage) {
        closedLoop = false;
        appliedVoltage = voltage;
    }
}
