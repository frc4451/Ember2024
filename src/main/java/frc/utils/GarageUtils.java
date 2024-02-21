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
     * @see {@link https://www.desmos.com/calculator/m49uyzuten}
     */
    public static double percentBoatCurve(double percent, double position, double min, double max) {
        double boatCurveFactor;
        if (position > min && Math.signum(percent) <= 0) {
            double positionMinDeltaSquared = Math.pow(position - min, 2);
            boatCurveFactor = (positionMinDeltaSquared - 1) / (positionMinDeltaSquared + 1);
        } else {
            if (position <= max && Math.signum(percent) > 0) {
                double maxPositionDifferenceSquared = Math.pow(max - position, 2);
                boatCurveFactor = (maxPositionDifferenceSquared - 1) / (maxPositionDifferenceSquared + 1);
            } else {
                boatCurveFactor = 1.0;
            }
        }
        return percent * boatCurveFactor;
    }
}
