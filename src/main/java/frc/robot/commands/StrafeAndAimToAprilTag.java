package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;

/**
 * Command for centering the robot relative to an April Tag that the user
 * specifies. The ID must be a valid Fiducial ID.
 */
public class StrafeAndAimToAprilTag extends Command {
    private static double yawMeasurementOffset = Math.PI; // To aim from the back
    private final PIDController thetaController = new PIDController(6, 0, 0.1);
    private final String logRoot;

    private final DriveSubsystem drive;
    private final int targetFiducialId;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private double yawErrorRad = yawMeasurementOffset;
    private Pose3d targetPose = new Pose3d();
    private boolean hasSeenTag = false;

    public StrafeAndAimToAprilTag(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            int targetFiducialId,
            DriveSubsystem drive) {
        addRequirements(drive);
        setName("StrafeAndAimToAprilTag");

        logRoot = "Commands/" + getName() + "/";

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.targetFiducialId = targetFiducialId;
        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.drive = drive;

        targetPose = VisionConstants.FIELD_LAYOUT.getTagPose(targetFiducialId).get();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        drive.runVelocity(new ChassisSpeeds());

        Logger.recordOutput(logRoot + "IsRunning", true);
    }

    @Override
    public void execute() {
        Pose3d robotPose = new Pose3d(drive.getPose());

        Set<TargetWithSource> targets = this.visibleAprilTagsSupplier.get();
        AprilTagAlgorithms.filterTags(targets.stream(), targetFiducialId)
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity()
                                        ? targetWithSourceA
                                        : targetWithSourceB)
                .ifPresent(
                        targetWithSource -> {
                            hasSeenTag = true;
                            Transform3d cameraToTarget = targetWithSource.target().getBestCameraToTarget();
                            Transform3d robotToCamera = targetWithSource.source().robotToCamera();
                            Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);
                            targetPose = robotPose.transformBy(robotToTarget);
                        });

        yawErrorRad = targetPose.relativeTo(robotPose).getTranslation().toTranslation2d().getAngle().getRadians();

        double rotationSpeedRad = thetaController.calculate(yawMeasurementOffset, yawErrorRad);

        double offsetYawRad = yawErrorRad + yawMeasurementOffset;

        Logger.recordOutput(logRoot + "TargetID", targetFiducialId);
        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "TargetYawRad", yawErrorRad);
        Logger.recordOutput(logRoot + "TargetYawDeg", Units.radiansToDegrees(yawErrorRad));
        Logger.recordOutput(logRoot + "OffsetYawRad", offsetYawRad);
        Logger.recordOutput(logRoot + "OffsetYawDeg", Units.radiansToDegrees(offsetYawRad));
        Logger.recordOutput(logRoot + "RotationSpeed", rotationSpeedRad);

        TeleopDrive.drive(
                drive,
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                rotationSpeedRad / DriveConstants.kMaxAngularSpeed,
                false,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(logRoot + "IsRunning", false);
    }
}
