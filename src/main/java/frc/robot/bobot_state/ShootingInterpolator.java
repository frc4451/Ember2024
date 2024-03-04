package frc.robot.bobot_state;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingInterpolator {
    public static record InterpolatedCalculation(
            double angleDegrees,
            double leftSpeedRotPerSec,
            double rightSpeedRotPerSec) {
    }

    public static record DistanceAngleSpeedEntry(
            double distanceMeters,
            double angleDegrees,
            double leftSpeedRotPerSec,
            double rightSpeedRotPerSec) {
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

    public void addEntries(DistanceAngleSpeedEntry... entries) {
        for (DistanceAngleSpeedEntry entry : entries) {
            addEntry(entry);
        }
    }

    public void addEntry(DistanceAngleSpeedEntry entry) {
        addDistanceAngleEntry(entry.distanceMeters, entry.angleDegrees);
        addLeftAngleSpeedEntry(entry.angleDegrees, entry.leftSpeedRotPerSec);
        addRightAngleSpeedEntry(entry.angleDegrees, entry.rightSpeedRotPerSec);
    }

    private void addDistanceAngleEntry(double distanceMeters, double angleDegrees) {
        distanceAngleMap.put(distanceMeters, angleDegrees);
    }

    private void addLeftAngleSpeedEntry(double angleDegrees, double leftSpeedRotPerSec) {
        leftAngleSpeedMap.put(angleDegrees, leftSpeedRotPerSec);
    }

    private void addRightAngleSpeedEntry(double angleDegrees, double rightSpeedRotPerSec) {
        rightAngleSpeedMap.put(angleDegrees, rightSpeedRotPerSec);
    }

    public InterpolatedCalculation calculateInterpolation(double distanceMeters) {
        double angleCalculation = distanceAngleMap.get(distanceMeters);
        double leftSpeedCalculation = leftAngleSpeedMap.get(angleCalculation);
        double rightSpeedCalculation = rightAngleSpeedMap.get(angleCalculation);

        return new InterpolatedCalculation(angleCalculation, leftSpeedCalculation, rightSpeedCalculation);
    }
}
