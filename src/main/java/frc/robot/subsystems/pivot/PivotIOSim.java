package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

// This doesn't work too well in some ways because it measures the angles differently than the real implementation.
// I also can't figure out how to implement the `setAngle` method for this.
// I think we could change the code in the real implementation to start at 0 instead of 184 degrees to make them line up.
// Won't be doing that right now but that might work.
public class PivotIOSim implements PivotIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            IntakeConstants.kPivotReduction,
            0.3, // moment of intertia (I think this is in kg * m^2) (number is wrong)
            0.8, // arm length (m) (number is wrong)
            0.0, // physical min (rad)
            Math.PI, // physical max (rad)
            false); // whether to simulate gravity

    private double appliedVoltage = 0.0;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        armSim.update(0.02); // 20 ms is the standard periodic loop time

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmperage = armSim.getCurrentDrawAmps();

        inputs.relativeAngleRad = armSim.getAngleRads();
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);

        appliedVoltage = voltage;

        armSim.setInputVoltage(voltage);
    }

    // Can't figure out how to do this
    // @Override
    // public void setAngle(Rotation2d angle) {}
}
