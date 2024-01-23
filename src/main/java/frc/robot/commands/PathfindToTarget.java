package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PathfindToTarget extends Command {
    private final PIDController xController = new PIDController(0.6, 0, 0);
    private final PIDController thetaController = new PIDController(1, 0, 0);

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Optional<PhotonTrackedTarget>> targetSupplier;

    private final DriveSubsystem drive;

    private boolean shouldFinish = false;

    public PathfindToTarget(
            Supplier<Pose2d> poseSupplier,
            Supplier<Optional<PhotonTrackedTarget>> targetSupplier,
            DriveSubsystem drive) {
        this.poseSupplier = poseSupplier;
        this.targetSupplier = targetSupplier;
        this.drive = drive;
    }

    @Override
    public void execute() {
        if (!targetSupplier.get().isPresent()) {
            drive.drive(0, 0, 0, false, true);
            shouldFinish = true;
            return;
        }

        PhotonTrackedTarget target = targetSupplier.get().get();

        Transform3d robotToCamera = VisionConstants.OBJECT_DETECTION_SOURCE.robotToCamera();

        double calculateDistanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(),
                Units.inchesToMeters(2),
                -robotToCamera.getRotation().getY(),
                Units.degreesToRadians(target.getPitch()));

        double x = xController.calculate(0, calculateDistanceToTargetMeters);
        double theta = thetaController.calculate(
                0,
                Units.degreesToRadians(target.getYaw()) + robotToCamera.getRotation().getZ());

        drive.drive(x, 0, theta, false, true);
    }

    @Override
    public boolean isFinished() {
        return shouldFinish;
    }
}
