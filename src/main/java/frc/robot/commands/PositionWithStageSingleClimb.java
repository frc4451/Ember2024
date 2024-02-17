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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;
import frc.robot.subsystems.vision.apriltag.StageTags;
import frc.utils.GarageUtils;

public class PositionWithStageSingleClimb extends Command {
    // private static double yawMeasurementOffset = Math.PI; // To aim from the back
    private final PIDController thetaController = new PIDController(6, 0, 0.1);
    private final PIDController xController = new PIDController(5, 0, 0);
    private final PIDController yController = new PIDController(5, 0, 0);

    private final String logRoot;

    private final DriveSubsystem drive;
    // private final int targetFiducialId;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;
    private final DoubleSupplier xSupplier;
    // private final DoubleSupplier ySupplier;
    private final StageTags stageTag;

    private Pose3d targetPose = new Pose3d();
    private boolean hasSeenTag = false;

    public PositionWithStageSingleClimb(
            DoubleSupplier xSupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            StageTags stageTag,
            DriveSubsystem drive) {

        addRequirements(drive);
        setName("PositionWithAmp");

        logRoot = "Commands/" + getName() + "/";

        this.xSupplier = xSupplier;

        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.drive = drive;

        this.stageTag = stageTag;

        targetPose = stageTag.getOffsetPose();

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
        AprilTagAlgorithms.filterTags(targets.stream(), this.stageTag.getId())
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

        // double xErrorMeters = targetPose.getX() - robotPose.getX();
        // double yErrorMeters = targetPose.getY() - robotPose.getY();

        ChassisSpeeds speed = new ChassisSpeeds(0, 0, 0);

        if (this.stageTag == StageTags.CENTER) {
            speed = aimCenterFieldFacingSide();
        } else if (this.stageTag == StageTags.AMP || this.stageTag == StageTags.HUMAN) {
            speed = aimAngledFacingSide();
        }

        double xSpeedMeters = MathUtil.clamp(
                xController.calculate(0, speed.vxMetersPerSecond),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        double ySpeedMeters = MathUtil.clamp(
                yController.calculate(0, speed.vyMetersPerSecond),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        Logger.recordOutput(logRoot + "TargetID", this.stageTag.getId());
        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "RotationSpeed", speed.omegaRadiansPerSecond);

        TeleopDrive.drive(
                drive,
                GarageUtils.getFlipped() * xSpeedMeters / DriveConstants.kMaxSpeedMetersPerSecond,
                GarageUtils.getFlipped() * ySpeedMeters / DriveConstants.kMaxSpeedMetersPerSecond,
                speed.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
                false,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(logRoot + "IsRunning", false);
    }

    /**
     * Handle Side of Stage closest to the center of the field.
     */
    private ChassisSpeeds aimCenterFieldFacingSide() {
        double rotationSpeedRad = thetaController.calculate(
                drive.getPose().getRotation().getRadians(),
                stageTag.getOffsetPose().getRotation().getAngle());

        double yErrorMeters = targetPose.getY() - drive.getPose().getY();

        return new ChassisSpeeds(-this.xSupplier.getAsDouble(), yErrorMeters, rotationSpeedRad);
    }

    /**
     * Handle Sides of Stage visible to the Driver Station.
     */
    private ChassisSpeeds aimAngledFacingSide() {
        double rotationSpeedRad = thetaController.calculate(
                drive.getPose().getRotation().getRadians(),
                stageTag.getOffsetPose().getRotation().getAngle());

        double threshold = 0.4;

        double x = xSupplier.getAsDouble();
        double y = xSupplier.getAsDouble();

        double xSpeedMeters = 0.0;
        double ySpeedMeters = 0.0;

        if (Math.abs(x) > threshold && Math.abs(y) > threshold) {
            xSpeedMeters = x * Math.cos(stageTag.getOffsetPose().getRotation().getAngle());
            ySpeedMeters = -y * Math.sin(stageTag.getOffsetPose().getRotation().getAngle());
        }

        return new ChassisSpeeds(-xSpeedMeters, ySpeedMeters, rotationSpeedRad);
    }
}
