package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    // private static double momentOfInertiaKgMSquared = 0.0000032998;
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim leftSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);
    private final FlywheelSim rightSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController leftPidController = new PIDController(1, 0, 0);
    private final PIDController rightPidController = new PIDController(1, 0, 0);

    private double velocityRotPerSecondLeft;
    private double velocityRotPerSecondRight;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double leftVoltage = 12.0
                * leftPidController.calculate(leftSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondLeft);
        double rightVoltage = 12.0
                * rightPidController.calculate(rightSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondRight);

        leftSim.setInputVoltage(leftVoltage);
        rightSim.setInputVoltage(rightVoltage);

        leftSim.update(0.02); // 20 ms is the standard periodic loop time
        rightSim.update(0.02);

        inputs.appliedVoltage = new double[] { leftVoltage, rightVoltage };
        inputs.currentAmperage = new double[] { leftSim.getCurrentDrawAmps(), rightSim.getCurrentDrawAmps() };
        inputs.velocityRotPerSecond = new double[] {
                leftSim.getAngularVelocityRPM() / 60.0,
                rightSim.getAngularVelocityRPM() / 60.0
        };
    }

    @Override
    public void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        this.velocityRotPerSecondLeft = velocityRotPerSecondLeft;
        this.velocityRotPerSecondRight = velocityRotPerSecondRight;
    }

    @Override
    public void setFree() {
        leftSim.setInputVoltage(0);
        rightSim.setInputVoltage(0);
    }

    @Override
    public void stop() {
        setVelocity(0.0, 0.0);
    }
}
