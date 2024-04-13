package frc.robot.bobot_state.interpolation;

import edu.wpi.first.math.util.Units;
import frc.robot.bobot_state.interpolation.ShootingInterpolator.DistanceAngleSpeedEntry;
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
                        Units.feetToMeters(25),
                        50.0,
                        50.0,
                        50.0));

        interpolator.addEntries(
                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(28),
                        50.0,
                        55.0,
                        55.0));

        interpolator.addEntries(
                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(30.5),
                        45,
                        57.5,
                        57.5));
    }

    public FloorInterpolator() {
        super(OffsetTags.FLOOR_SHOT, interpolator);
    }
}
