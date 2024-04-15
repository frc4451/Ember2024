package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.VisionConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;

/**
 * Command for centering the robot relative to an April Tag that the user
 * specifies. The ID must be a valid Fiducial ID.
 */
public class StrafeAndAimToAprilTag extends Command {
    private final PIDController thetaController = new PIDController(5, 0, 0.1);
    private final String logRoot;

    private final DriveSubsystem drive;
    private final int targetFiducialId;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final boolean fieldRelative;

    private final double yawMeasurementOffset;
    private double yawErrorRad;
    private Pose3d targetPose = new Pose3d();
    private boolean hasSeenTag = false;

    public StrafeAndAimToAprilTag(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            int targetFiducialId,
            DriveSubsystem drive,
            boolean fieldRelative) {
        this(xSupplier, ySupplier, targetFiducialId, drive, fieldRelative, Math.PI);
    }

    public StrafeAndAimToAprilTag(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            int targetFiducialId,
            DriveSubsystem drive,
            boolean fieldRelative,
            double yawMeasurementOffset) {
        addRequirements(drive);
        setName("StrafeAndAimToAprilTag");

        logRoot = "Commands/" + getName() + "/";

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.targetFiducialId = targetFiducialId;
        this.drive = drive;
        this.fieldRelative = fieldRelative;
        this.yawMeasurementOffset = yawMeasurementOffset;
        this.yawErrorRad = yawMeasurementOffset;

        targetPose = VisionConstants.FIELD_LAYOUT.getTagPose(targetFiducialId).get();

        thetaController.setTolerance(Units.degreesToRadians(0.1));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StrafeAndAimToAprilTag(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            int targetFiducialId,
            DriveSubsystem drive) {
        this(xSupplier, ySupplier, targetFiducialId, drive, true);
    }

    @Override
    public void initialize() {
        drive.runVelocity(new ChassisSpeeds());

        Logger.recordOutput(logRoot + "IsRunning", true);
    }

    @Override
    public void execute() {
        Pose3d robotPose = new Pose3d(drive.getPose());

        Set<TargetWithSource> targets = BobotState.getVisibleAprilTags();
        AprilTagAlgorithms.filterTags(targets.stream(), targetFiducialId)
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
                fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(logRoot + "IsRunning", false);
    }
}
