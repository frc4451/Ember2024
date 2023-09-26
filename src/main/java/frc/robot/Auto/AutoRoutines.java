package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

class PathPlannerGroups {
    public static List<PathPlannerTrajectory> TEST = PathPlanner.loadPathGroup(
            "test",
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared));
}

public class AutoRoutines {
    private RobotContainer robotContainer;

    private final SendableChooser<Command> chooser = new SendableChooser<>();

    public final HashMap<String, Command> routines = new HashMap<>();

    public AutoRoutines(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        this.routines.put("test", this.getTestRoutine());

        this.registerDashboard();
    }

    public void registerDashboard() {
        this.routines.forEach((String label, Command cmd) -> {
            chooser.addOption(label, cmd);
        });

        SmartDashboard.putData("Auto Routines", chooser);
    }

    public Command getTestRoutine() {
        return this.robotContainer.m_robotDrive.followTrajectoryCommand(PathPlannerGroups.TEST.get(0),
                true)
                .andThen(() -> {
                    robotContainer.m_robotDrive.drive(0, 0, 0, false, false);
                });
    }

    public Command getSelectedRoutine() {
        return this.chooser.getSelected();
    }
}
