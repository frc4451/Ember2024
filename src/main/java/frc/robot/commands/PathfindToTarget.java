package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PathfindToTarget extends Command {
    private final PIDController xController = new PIDController(0.6, 0, 0);
    private final PIDController thetaController = new PIDController(1, 0, 0);

    private final Supplier<Pose2d> poseSupplier;

    private final Supplier<Optional<PhotonTrackedTarget>> targetSupplier;

    private final DriveSubsystem drive;

    private boolean shouldFinish = false;

    private Pose2d endPose;

    private Command pathfindToPoseCommand;

    private static final Translation2d NOTE_SIZE = new Translation2d(
            Units.inchesToMeters(14),
            Units.inchesToMeters(14));

    public PathfindToTarget(
            Supplier<Pose2d> poseSupplier,
            Supplier<Optional<PhotonTrackedTarget>> targetSupplier,
            DriveSubsystem drive) {
        this.poseSupplier = poseSupplier;
        this.targetSupplier = targetSupplier;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, false, true);
    }

    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> maybeTarget = targetSupplier.get();
        if (pathfindToPoseCommand != null) {
            shouldFinish = pathfindToPoseCommand.isFinished();
            if (shouldFinish) {
                pathfindToPoseCommand.end(false);
            } else {
                pathfindToPoseCommand.execute();
            }
            return;
        } else if (!maybeTarget.isPresent()) {
            if (endPose != null) {
                drive.drive(0, 0, 0, false, false);
                pathfindToPoseCommand = AutoBuilder.pathfindToPose(
                        endPose,
                        Constants.PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS);
                pathfindToPoseCommand.initialize();
            } else {
                shouldFinish = true;
            }
            return;
        }

        Transform3d robotToCamera = VisionConstants.OBJECT_DETECTION_SOURCE.robotToCamera();

        PhotonTrackedTarget target = maybeTarget.get();

        // The distance in a straight line from the camera to the target
        double distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(),
                Units.inchesToMeters(2),
                -robotToCamera.getRotation().getY(),
                Units.degreesToRadians(target.getPitch()));

        // The translation to the target from the camera split into x and y
        Translation2d cameraToTarget = PhotonUtils.estimateCameraToTargetTranslation(
                distanceToTargetMeters,
                Rotation2d.fromDegrees(target.getYaw()))
        // .plus(NOTE_SIZE.times(distanceToTargetMeters))
        ;

        Pose2d pose = poseSupplier.get();

        Transform2d robotToTarget = new Transform2d(
                robotToCamera.getTranslation().toTranslation2d(),
                robotToCamera.getRotation().toRotation2d())
                .plus(
                        new Transform2d(cameraToTarget, cameraToTarget.getAngle()));

        endPose = pose
                .transformBy(robotToTarget);

        double x = xController.calculate(0, distanceToTargetMeters);
        double theta = thetaController.calculate(
                0,
                Units.degreesToRadians(target.getYaw()));

        drive.drive(-x, 0, theta, false, false);
    }

    @Override
    public boolean isFinished() {
        return shouldFinish;
    }
}
