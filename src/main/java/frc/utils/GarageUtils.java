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

    /**
     *
     * @param percent  Percent -1.0 to 1.0
     * @param position Current position in units
     * @param min      Min position
     * @param max      Max position
     * @return
     */
    public static double percentSmoothBetweenValues(double percent, double position, double min, double max) {
        double x = position;
        double y = percent;
        return y * (x > min && 0 > Math.signum(y)
                ? (Math.pow(x - min, 2) - 1) / (Math.pow(x - min, 2) + 1)
                : max > x && Math.signum(y) > 0
                        ? (Math.pow(max - x, 2) - 1) / (Math.pow(max - x - x, 2) + 1)
                        : 1.0);
    }
}
