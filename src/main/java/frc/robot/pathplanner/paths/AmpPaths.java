package frc.robot.pathplanner.paths;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathPlannerConstants;
import frc.utils.GarageUtils;

public class AmpPaths {
    public static Command getAmpPath() {
        Pose2d target = GarageUtils.getAlliance() == Alliance.Red
                ? new Pose2d(14.70, 8.0, Rotation2d.fromDegrees(90))
                : new Pose2d(1.84, 8.0, Rotation2d.fromDegrees(90));

        return AutoBuilder.pathfindToPose(
                target,
                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                0.0,
                0.0);
    }
}
