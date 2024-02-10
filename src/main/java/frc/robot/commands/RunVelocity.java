package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RunVelocity extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;

    public RunVelocity(
            DriveSubsystem drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        setName("RunVelocity");
        addRequirements(drive);

        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void execute() {
        drive.runVelocity(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                omegaSupplier.getAsDouble());
    }
}
