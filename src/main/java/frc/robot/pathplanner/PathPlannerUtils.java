package frc.robot.pathplanner;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

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

    private static Command getPathFromName(String name) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);

        return AutoBuilder.followPath(path);
    }

    /**
     * Gets the test path
     *
     * @return Command
     */
    public static Command getTestPath() {
        return getPathFromName("Test Path");
    }

    public static Command getPoopPath() {
        return getPathFromName("poop m2");
    }
}
