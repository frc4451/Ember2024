package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRoutines {
    public static Command getTestRoutine(DriveSubsystem drive, List<PathPlannerTrajectory> paths) {
        return drive.followTrajectoryCommand(paths.get(0), true, true);
    }

    public static Command getTriPoop(DriveSubsystem drive, List<PathPlannerTrajectory> paths) {
        final PathPlannerTrajectory goGrabSecond = paths.get(0);
        final PathPlannerTrajectory goPoopSecond = paths.get(1);
        final PathPlannerTrajectory goGrabThird = paths.get(2);
        final PathPlannerTrajectory goPoopThird = paths.get(3);

        return new SequentialCommandGroup(
                // poop first game piece
                drive.followTrajectoryCommand(goGrabSecond, true, true),
                // actually grab
                drive.followTrajectoryCommand(goPoopSecond, false, true),
                // poop second game piece
                drive.followTrajectoryCommand(goGrabThird, false, true),
                // actually grab third piece
                drive.followTrajectoryCommand(goPoopThird, false, true)
        // poop second game piece
        );
    }
}
