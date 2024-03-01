package frc.robot.pathplanner;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConstants;

public class PathPlannerUtils {
    public static void configureLogging() {
        // Configure PathPlanner for generative pathfinding
        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Pathplanner/Trajectory",
                            activePath.toArray(Pose2d[]::new));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Pathplanner/TrajectoryTarget", targetPose);
                });

        // Initial Values
        Logger.recordOutput("Pathplanner/Trajectory", new Pose2d[0]);
        Logger.recordOutput("Pathplanner/TrajectoryTarget", new Pose2d());
    }

    public static Command pathToPoseCommand(Pose2d pose) {
        return Commands.deferredProxy(
                () -> AutoBuilder.pathfindToPose(
                        pose,
                        PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                        0.0,
                        0.0));
    }
}
