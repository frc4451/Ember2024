package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
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
    private static double yawMeasurementOffset = Math.PI; // To aim from the back
    private final PIDController thetaController = new PIDController(6, 0, 0.1);

    private final DriveSubsystem drive;
    private final int targetFiducialId;
    private final AprilTag targetTag;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;

    private final DeltaTimeTracker gyroDeltaTracker = new DeltaTimeTracker();
    private double yawErrorRad = yawMeasurementOffset;
    private boolean hasSeenTag = false;

    public RotateShooterToAprilTag(
            DriveSubsystem drive,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            int targetFiducialId) {
        addRequirements(drive);
        setName("RotateShooterToAprilTag");

        this.drive = drive;
        this.targetFiducialId = targetFiducialId;
        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.targetTag = VisionConstants.FIELD_LAYOUT.getTags().get(targetFiducialId - 1);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(0.5));
    }

    @Override
    public void initialize() {
        drive.runVelocity(new ChassisSpeeds());
        gyroDeltaTracker.update(drive.getPose().getRotation().getRadians());
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
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity() ? targetWithSourceA : targetWithSourceB)
                .ifPresentOrElse(
                        targetWithSource -> {
                            hasSeenTag = true;
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
                        () -> {
                            if (hasSeenTag) {
                                yawErrorRad += gyroDeltaTracker.get();
                            } else {
                                // yawErrorRad = PhotonUtils.getYawToPose(
                                // drive.getPose(),
                                // targetTag.pose.toPose2d()).getRadians();
                            }
                        });

        double rotationSpeedRad = thetaController.calculate(yawMeasurementOffset, yawErrorRad);

        double offsetYawRad = yawErrorRad + yawMeasurementOffset;

        String logRoot = "Commands/" + getName() + "/";

        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "TargetYawRad", yawErrorRad);
        Logger.recordOutput(logRoot + "TargetYawDeg", Units.radiansToDegrees(yawErrorRad));
        Logger.recordOutput(logRoot + "OffsetYawRad", offsetYawRad);
        Logger.recordOutput(logRoot + "OffsetYawDeg", Units.radiansToDegrees(offsetYawRad));
        Logger.recordOutput(logRoot + "RotationSpeed", rotationSpeedRad);

        if (!thetaController.atSetpoint()) {
            drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeedRad));
        }
    }
}
