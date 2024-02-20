package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {
    private static final double kRotationsToInches = ClimberConstants.kClimberSpoolDiameter * Math.PI;

    private static final double momentOfInertiaKgMSquared = 0.05; // Moment of intertia (totally wrong)

    private final DCMotorSim sim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            ClimberConstants.kClimberReduction,
            momentOfInertiaKgMSquared);

    private double appliedVoltage = 0.0;

    public void updateInputs(ClimberIOInputs inputs) {
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
    public void setPercentOutput(double percentDecimal) {
        setVoltage(12.0 * percentDecimal);
    }

    @Override
    public void setPosition(double positionInches) {
        sim.setState(positionInches / kRotationsToInches, sim.getAngularVelocityRadPerSec());
    }
}
