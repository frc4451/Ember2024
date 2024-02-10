package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PathPlannerConstants;

public class GarageUtils {
    /**
     * Simpler way to get current alliance, or return our predetermined
     * "DEFAULT" alliance.
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
                : PathPlannerConstants.DEFAULT_ALLIANCE;
    }

    public static boolean isBlueAlliance() {
        return GarageUtils.getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return GarageUtils.getAlliance() == Alliance.Red;
    }

    public static double getFlipped() {
        return GarageUtils.isRedAlliance() ? -1 : 1;
    }
}
