package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    // private static double momentOfInertiaKgMSquared = 0.0000032998;
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim topSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);
    private final FlywheelSim bottomSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController topPidController = new PIDController(1, 0, 0);
    private final PIDController bottomPidController = new PIDController(1, 0, 0);

    private double velocityRotPerSecondTop;
    private double velocityRotPerSecondBottom;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double topVoltage = 12.0
                * topPidController.calculate(topSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondTop);
        double bottomVoltage = 12.0
                * bottomPidController.calculate(bottomSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondBottom);

        topSim.setInputVoltage(topVoltage);
        bottomSim.setInputVoltage(bottomVoltage);

        topSim.update(0.02); // 20 ms is the standard periodic loop time
        bottomSim.update(0.02);

        inputs.appliedVoltage = new double[] { topVoltage, bottomVoltage };
        inputs.currentAmperage = new double[] { topSim.getCurrentDrawAmps(), bottomSim.getCurrentDrawAmps() };
        inputs.velocityRotPerSecond = new double[] {
                topSim.getAngularVelocityRPM() / 60.0,
                bottomSim.getAngularVelocityRPM() / 60.0
        };
    }

    @Override
    public void setVelocity(double velocityRotPerSecondTop, double velocityRotPerSecondBottom) {
        this.velocityRotPerSecondTop = velocityRotPerSecondTop;
        this.velocityRotPerSecondBottom = velocityRotPerSecondBottom;
    }

    @Override
    public void setVoltage(double topVoltage, double bottomVoltage) {
        topSim.setInputVoltage(topVoltage);
        bottomSim.setInputVoltage(bottomVoltage);
    }
}
