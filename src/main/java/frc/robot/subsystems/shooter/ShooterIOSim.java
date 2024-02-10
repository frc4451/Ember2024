package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    // private static double momentOfInertiaKgMSquared = 0.0000032998;
    private static double momentOfInertiaKgMSquared = 1.0;

    private final FlywheelSim leftSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);
    private final FlywheelSim rightSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);
    private final FlywheelSim feederSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, momentOfInertiaKgMSquared);

    private final PIDController leftPidController = new PIDController(1, 0, 0);
    private final PIDController rightPidController = new PIDController(1, 0, 0);
    private final PIDController feederPidController = new PIDController(1, 0, 0);

    private double velocityRotPerSecondLeft = 0.0;
    private double velocityRotPerSecondRight = 0.0;
    private double velocityRotPerSecondFeeder = 0.0;

    private double appliedVoltageLeft = 0.0;
    private double appliedVoltageRight = 0.0;
    private double appliedVoltageFeeder = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        appliedVoltageLeft = 12.0
                * leftPidController.calculate(leftSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondLeft);
        appliedVoltageRight = 12.0
                * rightPidController.calculate(rightSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondRight);
        appliedVoltageFeeder = 12.0
                * feederPidController.calculate(leftSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecondFeeder);
        leftSim.setInputVoltage(appliedVoltageLeft);
        rightSim.setInputVoltage(appliedVoltageRight);
        feederSim.setInputVoltage(appliedVoltageRight);

        leftSim.update(0.02); // 20 ms is the standard periodic loop time
        rightSim.update(0.02);
        feederSim.update(0.02);

        inputs.appliedVoltageLeft = appliedVoltageLeft;
        inputs.appliedVoltageRight = appliedVoltageRight;
        inputs.appliedVoltageFeeder = appliedVoltageFeeder;

        inputs.currentAmperageLeft = leftSim.getCurrentDrawAmps();
        inputs.currentAmperageRight = rightSim.getCurrentDrawAmps();
        inputs.currentAmperageFeeder = feederSim.getCurrentDrawAmps();

        inputs.velocityRotPerSecondLeft = leftSim.getAngularVelocityRPM() / 60.0;
        inputs.velocityRotPerSecondRight = rightSim.getAngularVelocityRPM() / 60.0;
        inputs.velocityRotPerSecondFeeder = feederSim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight,
            double velocityRotPerSecondFeeder) {
        this.velocityRotPerSecondLeft = velocityRotPerSecondLeft;
        this.velocityRotPerSecondRight = velocityRotPerSecondRight;
        this.velocityRotPerSecondFeeder = velocityRotPerSecondFeeder;
    }

    @Override
    public void stopShooter() {
        leftSim.setInputVoltage(0);
        rightSim.setInputVoltage(0);
    }

    @Override
    public void stopFeeder() {
        feederSim.setInputVoltage(0);
    }
}
