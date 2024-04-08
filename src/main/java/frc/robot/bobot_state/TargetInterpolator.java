package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.bobot_state.ShootingInterpolator.InterpolatedCalculation;
import frc.robot.subsystems.vision.apriltag.OffsetTags;

public abstract class TargetInterpolator {
    private final OffsetTags tag;
    private final ShootingInterpolator interpolator;

    private Pose2d pose = new Pose2d();

    public TargetInterpolator(OffsetTags tag, ShootingInterpolator interpolator) {
        this.tag = tag;
        this.interpolator = interpolator;
    }

    public void update(Pose2d pose) {
        this.pose = pose;
    }

    public double getDistanceFromTarget() {
        return tag.getDistanceFrom(pose);
    }

    public InterpolatedCalculation calculateInterpolation() {
        return interpolator.calculateInterpolation(getDistanceFromTarget());
    }
}
