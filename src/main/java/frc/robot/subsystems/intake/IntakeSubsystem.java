package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;

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
        // this.io.updateInputs(this.inputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.io.stop();
        }

        // Logger.processInputs("Intake/Pivot", this.inputs);

        // this.angle = new Rotation2d(this.inputs.relativeAngleRad);

        // Logger.recordOutput("Pivot/Angle", getAngle().getDegrees());
        // Logger.recordOutput("Pivot/SetpointAngle", getSetpoint().getDegrees());
        // super.periodic();
    }

    public Command runPercentCommand(double topPercent, double bottmpercent) {
        return new RunCommand(() -> this.io.setVelocity(topPercent, bottmpercent));
    }
}
