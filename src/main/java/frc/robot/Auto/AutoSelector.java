package frc.robot.Auto;

import java.util.LinkedHashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

class Speeds {
    public static final PathConstraints THREE = new PathConstraints(3.0, 3.0);
}

class PathPlannerGroups {
    public static List<PathPlannerTrajectory> test = PathPlanner.loadPathGroup("test", Speeds.THREE);
    public static List<PathPlannerTrajectory> leftBlueTriPoop = PathPlanner.loadPathGroup(
            "leftBlueTriPoop",
            Speeds.THREE);
    public static List<PathPlannerTrajectory> rightBlueTriPoop = PathPlanner.loadPathGroup(
            "rightBlueTriPoop",
            Speeds.THREE);
}

public class AutoSelector {
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    private final LinkedHashMap<String, Command> routines = new LinkedHashMap<>();

    public AutoSelector(RobotContainer robotContainer) {
        this.routines.put("test", AutoRoutines.getTestRoutine(robotContainer.m_robotDrive, PathPlannerGroups.test));

        this.routines.put("Left Blue TriPoop",
                AutoRoutines.getTriPoop(
                        robotContainer.m_robotDrive,
                        PathPlannerGroups.leftBlueTriPoop));

        this.routines.put("Right Blue TriPoop",
                AutoRoutines.getTriPoop(
                        robotContainer.m_robotDrive,
                        PathPlannerGroups.rightBlueTriPoop));

        this.registerDashboard();
    }

    public void registerDashboard() {
        this.routines.forEach((String label, Command cmd) -> {
            chooser.addOption(label, cmd);
        });

        SmartDashboard.putData("Auto Routines", chooser);
    }

    public Command getSelectedRoutine() {
        return this.chooser.getSelected();
    }
}
