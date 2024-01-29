package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new IntakeIOTalonFX();
                break;
            case SIM:
                io = new IntakeIOSim();
                break;
            case REPLAY:
            default:
                io = new IntakeIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {

        this.io.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.io.setVelocity(0.0, 0.0);
        }
    }

    public Command setVelocityCommand(double topSpeed, double bottomSpeed) {
        return new InstantCommand(() -> this.io.setVelocity(topSpeed, bottomSpeed));
    }

    public Command stopCommand() {
        return new InstantCommand(() -> this.io.setVelocity(0.0, 0.0));
    }
}
