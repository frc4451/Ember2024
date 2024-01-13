package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

public class Paths {
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
