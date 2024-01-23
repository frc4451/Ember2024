package frc.robot.commands;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.VisionConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class PathfindToNoteCommand extends Command {
    public static Command pathFindToCommand(Optional<PhotonTrackedTarget> closestObject, Pose2d currentPose) {
        String loggingKey = "AutoBuilder/";

        if (closestObject.isEmpty()) {
            Logger.recordOutput(loggingKey + "Destination", new Pose2d());
            return Commands.none();
        }

        PhotonTrackedTarget target = closestObject.get();

        // We want to eventually be directly inline with the Note
        Rotation2d targetYaw = Rotation2d.fromDegrees(target.getYaw());

        // Position of Camera from Center of Robot
        Transform3d robotToCamera = VisionConstants.OBJECT_DETECTION_SOURCE.robotToCamera();

        Transform3d cameraToTarget = target.getBestCameraToTarget();

        Pose3d endPose = new Pose3d().transformBy(robotToCamera).transformBy(cameraToTarget);

        Logger.recordOutput("OUTPUT", endPose);

        return AutoBuilder.pathfindToPose(endPose.toPose2d(), PathPlannerConstants.TEST_PATH_CONSTRAINTS);
    }

    public static Command getPathfindToNoteCommand(VisionSubsystem vision, DriveSubsystem drive) {
        Optional<PhotonTrackedTarget> maybeClosestObject = vision.findClosestObject();
        if (maybeClosestObject.isPresent()) {
            // Please clean and fix this, it no worky
            PhotonTrackedTarget target = maybeClosestObject.get();
            Rotation2d targetYaw = Rotation2d.fromDegrees(target.getYaw());
            Transform3d transform3d = target.getBestCameraToTarget();
            Transform2d transform2d = new Transform2d(transform3d.getX(), transform3d.getY(),
                    transform3d.getRotation().toRotation2d());

            Pose2d endPose = drive.getPose().transformBy(transform2d);

            Logger.recordOutput("Debugg", true);

            // return new rotateInPlace(vision, drive);
            return Commands.none();
        } else {
            Logger.recordOutput("Debugg", false);
            return Commands.none();
        }
    }

    public static Command rotateInPlace(VisionSubsystem vision, DriveSubsystem drive) {
        return new RepeatCommand(new InstantCommand(() -> {
            PhotonTrackedTarget target = vision.findClosestObject().get();
            Rotation2d targetYaw = Rotation2d.fromDegrees(target.getYaw());
            drive.drive(targetYaw.getCos(), 0, targetYaw.getRadians() / Math.PI, false, true);
        }));
        // .until(() ->
        // MathUtil.applyDeadband(vision.findClosestObject().get().getYaw(), 0.5) == 0);
    }
}
