package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final PIDController pidController = new PIDController(
            Constants.ClimberConstants.kClimberP,
            Constants.ClimberConstants.kClimberI,
            Constants.ClimberConstants.kClimberD);

    public ClimberSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ClimberIOTalonFX(ClimberConstants.kClimberCanId, false);
                break;
            case SIM:
                io = new ClimberIOSim();
                break;
            case REPLAY:
            default:
                io = new ClimberIO() {
                };
                break;
        }

        this.reset();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Climber", this.inputs);

        if (DriverStation.isDisabled()) {
            this.io.stop();
        }
    }

    public void reset() {
        io.setPosition(0.0);
    }

    /**
     * GO TO SETPOINT :>
     *
     * @param setpoint
     */
    public void up(double setpoint) {
        this.io.setVelocity(this.pidController.calculate(this.io.getPosition(), setpoint));
    }

    public void down(double velocityPercent) {
        this.io.setVelocity(velocityPercent);
    }
}
