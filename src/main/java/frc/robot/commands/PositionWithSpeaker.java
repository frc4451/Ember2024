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
import frc.utils.GarageUtils;

public class PositionWithSpeaker extends Command {
    private static double yawMeasurementOffset = Math.PI; // To aim from the back
    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController thetaController = new PIDController(5, 0, 0.1);
    private final String logRoot;

    private final DriveSubsystem drive;
    private final StageTags tag;
    private final Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier;
    // private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private Pose3d tagPose = new Pose3d();
    private Pose3d targetPose = new Pose3d();
    private boolean hasSeenTag = false;

    public PositionWithSpeaker(
            DoubleSupplier ySupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            DriveSubsystem drive,
            StageTags tag) {
        addRequirements(drive);
        setName("PositionWithSpeaker");

        logRoot = "Commands/" + getName() + "/";

        this.ySupplier = ySupplier;

        this.tag = tag;

        this.visibleAprilTagsSupplier = visibleAprilTagsSupplier;
        this.drive = drive;

        xController.setTolerance(0.1);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        drive.runVelocity(new ChassisSpeeds());

        tagPose = tag.getPose();
        targetPose = tag.getOffsetPose();

        Logger.recordOutput(logRoot + "IsRunning", true);
    }

    @Override
    public void execute() {
        Pose3d robotPose = new Pose3d(drive.getPose());

        Set<TargetWithSource> targets = this.visibleAprilTagsSupplier.get();
        AprilTagAlgorithms.filterTags(targets.stream(), tag.getId())
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity()
                                        ? targetWithSourceA
                                        : targetWithSourceB)
                .ifPresent(
                        targetWithSource -> {
                            hasSeenTag = true;
                            tagPose = targetWithSource.getTargetPoseFrom(robotPose);
                            targetPose = tag.getOffsetPoseFrom(tagPose);
                        });

        double yawErrorRad = tagPose.relativeTo(robotPose)
                .getTranslation()
                .toTranslation2d()
                .getAngle()
                .getRadians();
        double rotationSpeedRad = thetaController.calculate(yawMeasurementOffset, yawErrorRad);

        double xSpeedMeters = MathUtil.clamp(
                xController.calculate(robotPose.getX(), targetPose.getX()),
                -DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxSpeedMetersPerSecond);

        Logger.recordOutput(logRoot + "TargetID", tag.getId());
        Logger.recordOutput(logRoot + "TagPose", tagPose);
        Logger.recordOutput(logRoot + "TagPose2d", tagPose.toPose2d());
        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "TargetPose2d", targetPose.toPose2d());
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
