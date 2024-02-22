package frc.robot.subsystems.pivot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class PivotInterpolator {
    public static record InterpolatedCalculation(
            double angleDegrees,
            double leftSpeedRotPerSec,
            double rightSpeedRotPerSec) {
    }

    public PivotInterpolator() {
    }

    /**
     * <p>
     * Lookup table for finding known good shots and guessing the angle we need
     * between those shots.
     * </p>
     * <p>
     * key: meters, values: degrees
     * </p>
     */
    private final InterpolatingDoubleTreeMap distanceAngleMap = new InterpolatingDoubleTreeMap();

    /**
     * key: angle (degrees), values: speed (rotations per second)
     */
    private final InterpolatingDoubleTreeMap leftAngleSpeedMap = new InterpolatingDoubleTreeMap();

    /**
     * key: angle (degrees), values: speed (rotations per second)
     */
    private final InterpolatingDoubleTreeMap rightAngleSpeedMap = new InterpolatingDoubleTreeMap();

    public void addDistanceAngleEntry(double distanceMeters, double angleDegrees) {
        distanceAngleMap.put(distanceMeters, angleDegrees);
    }

    public void addLeftAngleSpeedEntry(double angleDegrees, double leftSpeedRotPerSec) {
        leftAngleSpeedMap.put(angleDegrees, leftSpeedRotPerSec);
    }

    public void addRightAngleSpeedEntry(double angleDegrees, double rightSpeedRotPerSec) {
        rightAngleSpeedMap.put(angleDegrees, rightSpeedRotPerSec);
    }

    public InterpolatedCalculation calculateInterpolation(double distanceMeters) {
        double angleCalculation = distanceAngleMap.get(distanceMeters);
        double leftSpeedCalculation = leftAngleSpeedMap.get(angleCalculation);
        double rightSpeedCalculation = rightAngleSpeedMap.get(angleCalculation);

        return new InterpolatedCalculation(angleCalculation, leftSpeedCalculation, rightSpeedCalculation);
    }
}
