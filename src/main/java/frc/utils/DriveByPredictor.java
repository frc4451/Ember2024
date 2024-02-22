package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class DriveByPredictor {
    public record PivotAimingParameters(
            Rotation2d driveHeading,
            Rotation2d armAngle,
            double driveFeedVelocity) {
    }

    // Angle is in degrees and Distance is in ft
    private InterpolatingDoubleTreeMap angleDistanceFunction = new InterpolatingDoubleTreeMap();

    public void addMeasurement(double distance, double angle) {
        angleDistanceFunction.put(distance, angle);
    }

    /**
     * Calculates the aiming paramters needed through interpolation given the
     * distance
     *
     * @param distance
     * @return
     */
    public PivotAimingParameters calculate(double distance) {
        Rotation2d angle = Rotation2d.fromDegrees(angleDistanceFunction.get(distance));

        // This needs speed calculation, probably from another treemap, be and
        // driveHeading which will need to be determined by vision and account for
        // sideways velocity of the robot
        return new PivotAimingParameters(null, angle, 0.0);
    }
}
