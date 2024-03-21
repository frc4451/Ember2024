package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.GeomUtils;

/**
 * Handles routine for aiming at a note while driving.
 *
 * This should handle automatically rotating the robot's holonomic drive where
 * the YAW of the robot relative to a note is equal to zero.
 *
 * Meaning the robot should just drive forward and pick it up.
 *
 * By default, if not note is detected, it will rely on the driver's rotation
 * control.
 */
public class AimAtNote extends Command {
    private final PIDController thetaController = new PIDController(3, 0, 0);

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier thetaSupplier;

    private final Supplier<Optional<PhotonTrackedTarget>> targetSupplier;

    private final DriveSubsystem drive;

    public AimAtNote(
            Supplier<Optional<PhotonTrackedTarget>> targetSupplier,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier thetaSupplier,
            DriveSubsystem drive) {
        addRequirements(drive);
        setName("AimAtNote");

        this.targetSupplier = targetSupplier;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;
        this.drive = drive;

        thetaController.setTolerance(0.1);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> maybeTarget = targetSupplier.get();

        // If there are no targets, follow standard drive controls.
        if (!maybeTarget.isPresent()) {
            TeleopDrive.drive(
                    drive,
                    xSupplier.getAsDouble(),
                    ySupplier.getAsDouble(),
                    thetaSupplier.getAsDouble(),
                    false,
                    false);
            return;
        }

        PhotonTrackedTarget target = maybeTarget.get();

        double robotToTargetRadians = GeomUtils.getTransformFromNote(target).getRotation().getRadians();

        double theta = thetaController.calculate(
                robotToTargetRadians,
                0);

        TeleopDrive.drive(
                drive,
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                theta / DriveConstants.kMaxAngularSpeed,
                false,
                false);
    }
}
