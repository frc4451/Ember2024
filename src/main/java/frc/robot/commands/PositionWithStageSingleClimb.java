package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;
import frc.robot.subsystems.vision.apriltag.StageTags;

public class PositionWithStageSingleClimb extends Command {
    // private static double yawMeasurementOffset = Math.PI; // To aim from the back
    private final PIDController thetaController = new PIDController(6, 0, 0.1);
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
        setName("PositionWithStageSingleClimb");

        logRoot = "Commands/" + getName() + "/";

        this.xSupplier = xSupplier;

        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.drive = drive;

        this.stageTag = stageTag;
        targetPose = stageTag.getFieldPose();

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

        Set<TargetWithSource> targets = visibleAprilTagsSupplier.get();
        AprilTagAlgorithms.filterTags(targets.stream(), stageTag.getId())
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity()
                                        ? targetWithSourceA
                                        : targetWithSourceB)
                .ifPresent(
                        targetWithSource -> {
                            hasSeenTag = true;
                            targetPose = targetWithSource.getTargetPoseFrom(robotPose);
                        });

        ChassisSpeeds speeds = calculateCenteringSpeeds();

        Logger.recordOutput(logRoot + "TargetID", stageTag.getId());
        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "RotationSpeed", speeds.omegaRadiansPerSecond);

        TeleopDrive.drive(drive, speeds, true, false);
    }

    /**
     * Handle Side of Stage closest to the center of the field.
     */
    private ChassisSpeeds calculateCenteringSpeeds() {
        double rotationSpeedRad = thetaController.calculate(
                drive.getPose().getRotation().getRadians(),
                stageTag.getPathfindingPose().getRotation().getRadians());

        double xSpeedMeters = MathUtil.clamp(
                xSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        // double yErrorMeters = drive.getPose().getY() - targetPose.getY();
        double yErrorMeters = targetPose.toPose2d().relativeTo(drive.getPose())
                .getTranslation()
                .getY();

        double ySpeedMeters = MathUtil.clamp(
                yController.calculate(yErrorMeters, 0),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        return new ChassisSpeeds(-xSpeedMeters, -ySpeedMeters, rotationSpeedRad);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(logRoot + "IsRunning", false);
    }
}
