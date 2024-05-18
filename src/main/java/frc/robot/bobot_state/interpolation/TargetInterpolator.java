package frc.robot.bobot_state.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.bobot_state.interpolation.ShootingInterpolator.InterpolatedCalculation;
import frc.robot.subsystems.vision.apriltag.OffsetTags;

public abstract class TargetInterpolator {
    private final OffsetTags tag;
    private final ShootingInterpolator interpolator;

    private Pose2d pose = new Pose2d();
    private InterpolatedCalculation calculation = new InterpolatedCalculation(26.0, 0.0, 0.0);

    public TargetInterpolator(OffsetTags tag, ShootingInterpolator interpolator) {
        this.tag = tag;
        this.interpolator = interpolator;
    }

    public void update(Pose2d pose) {
        this.pose = pose;
        this.calculation = interpolator.calculateInterpolation(getDistanceFromTarget());
    }

    public double getDistanceFromTarget() {
        return this.tag.getDistanceFrom(pose);
    }

    public InterpolatedCalculation getCalculation() {
        return this.calculation;
    }
    // public InterpolatedCalculation calculateInterpolation() {
    // return interpolator.calculateInterpolation(getDistanceFromTarget());
    // }
}
