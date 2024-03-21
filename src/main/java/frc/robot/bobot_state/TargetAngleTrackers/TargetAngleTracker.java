package frc.robot.bobot_state.TargetAngleTrackers;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TargetAngleTracker {
    public abstract void update();

    public abstract Rotation2d getRotationDifference();
}
