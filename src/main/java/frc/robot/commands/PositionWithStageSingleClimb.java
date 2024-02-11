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
import frc.utils.PathfindPIDCalculation;

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
        targetPose = new Pose3d(stageTag.getPathfindingPose());
        // targetPose = VisionConstants.FIELD_LAYOUT.getTagPose(targetFiducialId).get();

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

        PathfindPIDCalculation calc = new PathfindPIDCalculation(0, 0, 0);

        if (this.stageTag == StageTags.CENTER) {
            calc = aimCenterFieldFacingSide();
        } else if (this.stageTag == StageTags.AMP || this.stageTag == StageTags.HUMAN) {
            calc = aimAngledFacingSide();
        }

        double xSpeedMeters = MathUtil.clamp(
                xController.calculate(0, calc.xSpeedMeters()),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        double ySpeedMeters = MathUtil.clamp(
                yController.calculate(0, calc.ySpeedMeters()),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        Logger.recordOutput(logRoot + "TargetID", this.stageTag.getId());
        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "RotationSpeed", calc.rotationSpeedRad());

        TeleopDrive.drive(
                drive,
                GarageUtils.getFlipped() * xSpeedMeters / DriveConstants.kMaxSpeedMetersPerSecond,
                GarageUtils.getFlipped() * ySpeedMeters / DriveConstants.kMaxSpeedMetersPerSecond,
                calc.rotationSpeedRad() / DriveConstants.kMaxAngularSpeed,
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
    private PathfindPIDCalculation aimCenterFieldFacingSide() {
        double rotationSpeedRad = thetaController.calculate(
                drive.getPose().getRotation().getRadians(),
                stageTag.getPathfindingPose().getRotation().getRadians());

        double yErrorMeters = targetPose.getY() - drive.getPose().getY();

        return new PathfindPIDCalculation(-this.xSupplier.getAsDouble(), yErrorMeters, rotationSpeedRad);
    }

    /**
     * Handle Sides of Stage visible to the Driver Station.
     */
    private PathfindPIDCalculation aimAngledFacingSide() {
        double rotationSpeedRad = thetaController.calculate(
                drive.getPose().getRotation().getRadians(),
                stageTag.getPathfindingPose().getRotation().getRadians());

        double threshold = 0.4;

        double x = xSupplier.getAsDouble();
        double y = xSupplier.getAsDouble();

        double xSpeedMeters = 0.0;
        double ySpeedMeters = 0.0;

        if (Math.abs(x) > threshold && Math.abs(y) > threshold) {
            xSpeedMeters = x * Math.cos(stageTag.getPathfindingPose().getRotation().getRadians());
            ySpeedMeters = y * -Math.sin(stageTag.getPathfindingPose().getRotation().getRadians());
        }

        return new PathfindPIDCalculation(-xSpeedMeters, ySpeedMeters, rotationSpeedRad);
    }
}
