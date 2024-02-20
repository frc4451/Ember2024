package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private static final double kRotationsToInches = ElevatorConstants.kElevatorSpoolDiameter * Math.PI;

    private static final double momentOfInertiaKgMSquared = 0.05; // Moment of intertia (totally wrong)

    private final DCMotorSim sim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            ElevatorConstants.kElevatorReduction,
            momentOfInertiaKgMSquared);

    private double appliedVoltage = 0.0;

    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmperage = sim.getCurrentDrawAmps();
        inputs.velocityInchesPerSecond = sim.getAngularVelocityRPM() / 60 * kRotationsToInches;
        inputs.positionInches = sim.getAngularPositionRotations() * kRotationsToInches;
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        sim.setInputVoltage(voltage);
    }

    @Override
    public void setPosition(double positionInches) {
        sim.setState(positionInches / kRotationsToInches, sim.getAngularVelocityRadPerSec());
    }
}
