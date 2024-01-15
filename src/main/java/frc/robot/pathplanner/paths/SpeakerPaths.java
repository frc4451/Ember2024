package frc.robot.pathplanner.paths;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathPlannerConstants;
import frc.utils.GarageUtils;

public class SpeakerPaths {
        public static Command getSpeakerCenter() {
                Pose2d target = GarageUtils.getAlliance() == Alliance.Red
                                ? new Pose2d(14.33, 5.5, Rotation2d.fromDegrees(0))
                                : new Pose2d(1.83, 5.5, Rotation2d.fromDegrees(180));

                return AutoBuilder.pathfindToPose(
                                target,
                                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                                0.0,
                                0.0);
        }

        public static Command getSpeakerLeftPath() {
                Pose2d target = GarageUtils.getAlliance() == Alliance.Red
                                ? new Pose2d(14.33, 6.0, Rotation2d.fromDegrees(-30))
                                : new Pose2d(1.83, 6.0, Rotation2d.fromDegrees(-150));

                return AutoBuilder.pathfindToPose(
                                target,
                                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                                0.0,
                                0.0);
        }

        public static Command getSpeakerRightPath() {
                Pose2d target = GarageUtils.getAlliance() == Alliance.Red
                                ? new Pose2d(14.33, 5.0, Rotation2d.fromDegrees(30))
                                : new Pose2d(1.83, 5.0, Rotation2d.fromDegrees(150));

                return AutoBuilder.pathfindToPose(
                                target,
                                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                                0.0,
                                0.0);
        }
}
