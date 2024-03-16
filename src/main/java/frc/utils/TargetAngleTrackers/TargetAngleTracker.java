package frc.utils.TargetAngleTrackers;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TargetAngleTracker {
    public abstract void update();

    public abstract Rotation2d getRotationDifference();

    public abstract void log();
}
