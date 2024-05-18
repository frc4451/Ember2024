package frc.robot.bobot_state.TargetAngleTrackers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TargetAngleTracker {
    public abstract void update();

    public abstract Optional<Rotation2d> getRotationTarget();
}
