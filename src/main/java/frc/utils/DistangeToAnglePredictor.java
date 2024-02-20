package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class DistangeToAnglePredictor {
    public record PivotAimingParameters(
            Rotation2d driveHeading,
            Rotation2d armAngle,
            double driveFeedVelocity) {
    }

    InterpolatingDoubleTreeMap m_interpolatingMap;

    /**
     * Calculates the aiming paramters needed through interpolation given the
     * distance
     *
     * @param distance
     * @return
     */
    public PivotAimingParameters calculate(double distance) {
        return new PivotAimingParameters(null, null, 0.0);
    }
}
