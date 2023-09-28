package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRoutines {
    public static Command getTestRoutine(DriveSubsystem drive, List<PathPlannerTrajectory> paths) {
        return drive.followTrajectoryCommand(paths.get(0), true, true);
    }
}
