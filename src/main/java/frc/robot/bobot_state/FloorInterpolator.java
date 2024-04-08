package frc.robot.bobot_state;

import edu.wpi.first.math.util.Units;
import frc.robot.bobot_state.ShootingInterpolator.DistanceAngleSpeedEntry;
import frc.robot.subsystems.vision.apriltag.OffsetTags;

public class FloorInterpolator extends TargetInterpolator {
    private final static ShootingInterpolator interpolator = new ShootingInterpolator(
            0,
            0,
            0,
            0);

    static {
        interpolator.addEntries(
                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(0),
                        26.9,
                        88.0,
                        73.0));
    }

    public FloorInterpolator() {
        super(OffsetTags.FLOOR_SHOT, interpolator);
    }
}
