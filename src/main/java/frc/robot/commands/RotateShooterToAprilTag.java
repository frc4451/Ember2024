package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;

/**
 * Command for centering the robot relative to an April Tag that the user
 * specifies. The ID must be a valid Fiducial ID.
 */
public class RotateShooterToAprilTag extends Command {
    private final PIDController thetaController = new PIDController(1, 0, 0);

    private final DriveSubsystem drive;
    private final int targetFiducialId;
    private final AprilTag targetTag;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;

    public RotateShooterToAprilTag(
            DriveSubsystem drive,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            int targetFiducialId) {
        this.drive = drive;
        this.targetFiducialId = targetFiducialId;
        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.targetTag = VisionConstants.FIELD_LAYOUT.getTags().get(targetFiducialId);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        // double targetYaw = PhotonUtils.getYawToPose(
        // drive.getPose(),
        // this.targetTag.pose.toPose2d())
        // // Shoot out the back?
        // .rotateBy(Rotation2d.fromRadians(Math.PI))
        // .getRadians();

        Set<TargetWithSource> targets = this.visibleAprilTagsSupplier.get();
        double targetYawRad = targets.stream()
                .filter(targetWithSource -> targetWithSource.target().getFiducialId() == targetFiducialId)
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity()
                                        ? targetWithSourceA
                                        : targetWithSourceB)
                // Assuming we have an actual AprilTag
                .map(targetWithSource -> new Rotation2d(targetWithSource.getYawRobotRelativeRad()))
                // Fallback to the AprilTag's field pose if we don't
                .orElseGet(() -> PhotonUtils.getYawToPose(drive.getPose(), targetTag.pose.toPose2d()))
                // To shoot out the back
                .rotateBy(new Rotation2d(Math.PI))
                .getRadians();

        double rotationSpeed = thetaController.calculate(0, targetYawRad);

        // @NOTE Use this for debugging, remove later
        Logger.recordOutput("RotateToAprilTag/TargetYaw", targetYawRad);
        Logger.recordOutput("RotateToAprilTag/RotationSpeed", rotationSpeed);

        drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
    }
}
