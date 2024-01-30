package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.utils.DeltaTimeTracker;

/**
 * Command for centering the robot relative to an April Tag that the user
 * specifies. The ID must be a valid Fiducial ID.
 */
public class RotateShooterToAprilTag extends Command {
    private final PIDController thetaController = new PIDController(5, 0, 0);

    private final DriveSubsystem drive;
    private final int targetFiducialId;
    private final AprilTag targetTag;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;

    private final DeltaTimeTracker gyroDeltaTracker = new DeltaTimeTracker();
    private double yawErrorRad = 0.0;

    public RotateShooterToAprilTag(
            DriveSubsystem drive,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            int targetFiducialId) {
        addRequirements(drive);

        this.drive = drive;
        this.targetFiducialId = targetFiducialId;
        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.targetTag = VisionConstants.FIELD_LAYOUT.getTags().get(targetFiducialId);

        gyroDeltaTracker.update(drive.getPose().getRotation().getRadians());
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

        gyroDeltaTracker.update(drive.getPose().getRotation().getRadians());

        Set<TargetWithSource> targets = this.visibleAprilTagsSupplier.get();
        targets.stream()
                .filter(targetWithSource -> targetWithSource.target().getFiducialId() == targetFiducialId)
                .findFirst()
                .ifPresentOrElse(
                        targetWithSource -> {
                            // yawErrorRad = Units
                            // .degreesToRadians(targetWithSource.getYawRobotRelativeRad()
                            Translation2d cameraToTarget = targetWithSource.target().getBestCameraToTarget()
                                    .getTranslation().toTranslation2d();
                            Transform3d robotToCamera = targetWithSource.source().robotToCamera();
                            Transform2d robotToTarget = new Transform2d(
                                    robotToCamera.getTranslation().toTranslation2d().plus(cameraToTarget),
                                    robotToCamera.getRotation().toRotation2d().plus(cameraToTarget.getAngle()));
                            yawErrorRad = robotToTarget.getRotation().getRadians();
                        },
                        () -> yawErrorRad += gyroDeltaTracker.get());
        // double targetYawRad = targets.stream()
        // .filter(targetWithSource -> targetWithSource.target().getFiducialId() ==
        // targetFiducialId)
        // .reduce((targetWithSourceA,
        // targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <=
        // targetWithSourceB
        // .target().getPoseAmbiguity()
        // ? targetWithSourceA
        // : targetWithSourceB)
        // // Assuming we have an actual AprilTag
        // .map(targetWithSource ->
        // Units.degreesToRadians(-targetWithSource.target().getYaw()))
        // // .orElseGet(() -> PhotonUtils.getYawToPose(drive.getPose(),
        // // targetTag.pose.toPose2d()))
        // .orElseGet(() -> Double.NaN);

        // if (Double.isNaN(targetYawRad)) {
        // return;
        // }

        double rotationSpeed = thetaController.calculate(Math.PI, yawErrorRad);

        // @NOTE Use this for debugging, remove later
        Logger.recordOutput("RotateToAprilTag/TargetYaw", yawErrorRad);
        Logger.recordOutput("RotateToAprilTag/RotationSpeed", rotationSpeed);

        drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
    }
}
