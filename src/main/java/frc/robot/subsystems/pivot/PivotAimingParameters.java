package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public record PivotAimingParameters(
        Rotation2d driveHeading,
        Rotation2d armAngle,
        double driveFeedVelocity) {
}
