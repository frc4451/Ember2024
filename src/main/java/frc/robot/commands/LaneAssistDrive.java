package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public abstract class LaneAssistDrive extends Command {
    protected final PIDController xController = new PIDController(5, 0, 0);
    protected final PIDController yController = new PIDController(5, 0, 0);
    protected final PIDController thetaController = new PIDController(5, 0, 0);

    abstract protected DriveSubsystem getDrive();

    abstract protected boolean isFieldRelative();

    abstract protected double getXOutput();

    abstract protected double getYOutput();

    abstract protected double getThetaOutput();

    protected void drive() {
        TeleopDrive.drive(
                getDrive(),
                getXOutput(),
                getYOutput(),
                getThetaOutput(),
                false,
                isFieldRelative());
    }
}
