package frc.robot.bobot_state.interpolation;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.pivot.PivotLocation;
import frc.utils.GarageUtils;

public class ShootingInterpolator {
    private final double blueCloseAngleFudgeFactor;
    private final double blueFarAngleFudgeFactor;
    private final double redCloseAngleFudgeFactor;
    private final double redFarAngleFudgeFactor;

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

    public ShootingInterpolator(
            double blueCloseAngleFudgeFactor,
            double blueFarAngleFudgeFactor,
            double redCloseAngleFudgeFactor,
            double redFarAngleFudgeFactor) {
        this.blueCloseAngleFudgeFactor = blueCloseAngleFudgeFactor;
        this.blueFarAngleFudgeFactor = blueFarAngleFudgeFactor;
        this.redCloseAngleFudgeFactor = redCloseAngleFudgeFactor;
        this.redFarAngleFudgeFactor = redFarAngleFudgeFactor;
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
    private final InterpolatingDoubleTreeMap blueDistanceAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap redDistanceAngleMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap distanceLeftSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceRightSpeedMap = new InterpolatingDoubleTreeMap();

    public void addEntries(DistanceAngleSpeedEntry... entries) {
        for (DistanceAngleSpeedEntry entry : entries) {
            addEntry(entry);
        }
    }

    public void addEntry(DistanceAngleSpeedEntry entry) {
        boolean isClose = entry.distanceMeters < 12;

        double blueFudgeFactor = isClose ? blueCloseAngleFudgeFactor : blueFarAngleFudgeFactor;

        double redFudgeFactor = isClose ? redCloseAngleFudgeFactor : redFarAngleFudgeFactor;

        double maxAngle = PivotLocation.kSoftMax.angle.getDegrees();

        blueDistanceAngleMap.put(
                entry.distanceMeters,
                Math.min(entry.angleDegrees + blueFudgeFactor, maxAngle));

        redDistanceAngleMap.put(
                entry.distanceMeters,
                Math.min(entry.angleDegrees + redFudgeFactor, maxAngle));

        distanceLeftSpeedMap.put(entry.distanceMeters, entry.leftSpeedRotPerSec);

        distanceRightSpeedMap.put(entry.distanceMeters, entry.rightSpeedRotPerSec);
    }

    public InterpolatedCalculation calculateInterpolation(double distanceMeters) {
        InterpolatingDoubleTreeMap angleMap = GarageUtils.isBlueAlliance()
                ? blueDistanceAngleMap
                : redDistanceAngleMap;

        double angleCalculation = angleMap.get(distanceMeters);
        double leftSpeedCalculation = distanceLeftSpeedMap.get(distanceMeters);
        double rightSpeedCalculation = distanceRightSpeedMap.get(distanceMeters);

        return new InterpolatedCalculation(angleCalculation, leftSpeedCalculation, rightSpeedCalculation);
    }
}
