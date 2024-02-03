package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;
import frc.utils.GarageUtils;

public class PositionWithAmp extends Command {
    private static double yawMeasurementOffset = Math.PI; // To aim from the back
    private final PIDController thetaController = new PIDController(6, 0, 0.1);
    private final PIDController xController = new PIDController(5, 0, 0);
    private final String logRoot;

    private final DriveSubsystem drive;
    private final int targetFiducialId;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;
    // private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private Pose3d targetPose = new Pose3d();
    private boolean hasSeenTag = false;

    public PositionWithAmp(
            DoubleSupplier ySupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            DriveSubsystem drive) {
        addRequirements(drive);
        setName("PositionWithAmp");

        logRoot = "Commands/" + getName() + "/";

        this.ySupplier = ySupplier;
        this.targetFiducialId = GarageUtils.isBlueAlliance()
                ? VisionConstants.BLUE_AMP_TAG
                : VisionConstants.RED_AMP_TAG;
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

        double xErrorMeters = targetPose.getX() - robotPose.getX();
        double rotationSpeedRad;
        if (Math.abs(xErrorMeters) < 1.0) {
            double yawErrorRad = targetPose.relativeTo(robotPose)
                    .getTranslation()
                    .toTranslation2d()
                    .getAngle()
                    .getRadians();
            rotationSpeedRad = thetaController.calculate(yawMeasurementOffset, yawErrorRad);
        } else {
            rotationSpeedRad = thetaController.calculate(
                    drive.getPose().getRotation().getRadians(),
                    -Math.PI / 2);
        }

        double xSpeedMeters = MathUtil.clamp(
                xController.calculate(0, xErrorMeters),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        Logger.recordOutput(logRoot + "TargetID", targetFiducialId);
        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "RotationSpeed", rotationSpeedRad);

        TeleopDrive.drive(
                drive,
                GarageUtils.getFlipped() * xSpeedMeters / DriveConstants.kMaxSpeedMetersPerSecond,
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
