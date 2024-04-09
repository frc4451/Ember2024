package frc.robot.bobot_state.interpolation;

import edu.wpi.first.math.util.Units;
import frc.robot.bobot_state.interpolation.ShootingInterpolator.DistanceAngleSpeedEntry;
import frc.robot.subsystems.vision.apriltag.OffsetTags;

public class SpeakerInterpolator extends TargetInterpolator {
    private static final ShootingInterpolator interpolator = new ShootingInterpolator(
            1.0,
            1.0,
            1.0,
            1.0);

    private static final double kLeftShooterSpeed = 88.0;

    private static final double kRightShooterSpeed = 73.0;

    static {
        interpolator.addEntries(
                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(7),
                        42.0,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(8),
                        40.0,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(9),
                        37.0,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(10),
                        34.0,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(11),
                        31.5,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(12),
                        30,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(13),
                        28.5,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(14),
                        27.5,
                        kLeftShooterSpeed,
                        kRightShooterSpeed),

                new DistanceAngleSpeedEntry(
                        Units.feetToMeters(15),
                        26.9,
                        kLeftShooterSpeed,
                        kRightShooterSpeed));
    }

    public SpeakerInterpolator() {
        super(OffsetTags.SPEAKER_AIM, interpolator);
    }
}
