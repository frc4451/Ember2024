package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class PivotIOSim implements PivotIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            PivotConstants.kPivotReduction,
            0.3, // moment of intertia (I think this is in kg * m^2) (number is wrong)
            0.8, // arm length (m) (number is wrong)
            PivotLocation.INITIAL.angle.getRadians(), // physical min (rad)
            Math.PI, // physical max (rad)
            false, // whether to simulate gravity
            PivotLocation.INITIAL.angle.getRadians()); // starting angle

    private final PIDController pidController = new PIDController(1, 0, 0);

    // private double velocityRadPerSecond = 0.0;
    private double positionRad = 0.0;

    private double appliedVoltage = 0.0;

    private boolean closedLoop = false;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (closedLoop) {
            // appliedVoltage = 12.0
            // * pidController.calculate(
            // armSim.getVelocityRadPerSec(),
            // velocityRadPerSecond);
            appliedVoltage = 12.0
                    * pidController.calculate(
                            armSim.getAngleRads(),
                            positionRad);
        }

        armSim.setInputVoltage(appliedVoltage);
        armSim.update(Constants.loopback); // 20 ms is the standard periodic loop time

        inputs.appliedVoltageLeader = appliedVoltage;
        inputs.appliedVoltageFollower = appliedVoltage;

        inputs.currentAmperageLeader = armSim.getCurrentDrawAmps();
        inputs.currentAmperageFollower = inputs.currentAmperageLeader;

        inputs.positionRadLeader = armSim.getAngleRads();
        inputs.positionRadFollower = inputs.positionRadLeader;

        inputs.velocityRadPerSecLeader = armSim.getVelocityRadPerSec();
        inputs.velocityRadPerSecFollower = inputs.velocityRadPerSecLeader;
    }

    @Override
    public void setVoltage(double voltage) {
        closedLoop = false;
        appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    @Override
    public void setPosition(double positionRad) {
        closedLoop = true;
        this.positionRad = positionRad;
    }

    @Override
    public void setPercentOutput(double percentDecimal) {
        setVoltage(12.0 * percentDecimal);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        armSim.setState(angle.getRadians(), armSim.getVelocityRadPerSec());
    }
}
