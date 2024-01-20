package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class PathfindToNoteCommand extends Command {
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

            return new SequentialCommandGroup(
                    rotateInPlace(vision, drive),
                    AutoBuilder.pathfindToPose(endPose, PathPlannerConstants.TEST_PATH_CONSTRAINTS));
        } else {
            Logger.recordOutput("Debugg", false);
            return Commands.none();
        }
    }

    public static Command rotateInPlace(VisionSubsystem vision, DriveSubsystem drive) {
        return new RunCommand(() -> {
            PhotonTrackedTarget target = vision.findClosestObject().get();
            drive.rotateInPlace(target.getYaw() / 180.0);
        })
                .until(() -> MathUtil.applyDeadband(vision.findClosestObject().get().getYaw(), 0.5) == 0);
    }
}
