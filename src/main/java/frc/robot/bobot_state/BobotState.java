package frc.robot.bobot_state;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.OffsetTags;
import frc.utils.VirtualSubsystem;

/**
 * Class full of static variables and methods that store robot state we'd need
 * across mulitple subsystems. It's called `BobotState` as to not conflict with
 * WPILib's {@link edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
    private static final String logRoot = "BobotState/";

    private static final ShootingInterpolator shootingInterpolator = new ShootingInterpolator();

    private static ShootingInterpolator.InterpolatedCalculation shootingCalculation;

    private static Pose2d robotPose = new Pose2d();

    private static Set<TargetWithSource> visibleAprilTags = new HashSet<>();

    private static boolean isElevatorUp = false;

    static {
        shootingInterpolator.addEntries(
                // // Subwoofer (calculated: 92cm from wall)
                // new ShootingInterpolator.DistanceAngleSpeedEntry(
                // 1.36,
                // 55.0,
                // 70.0,
                // 50.0),
                // 10ft

                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Double.MIN_VALUE,
                        36.0,
                        85.0,
                        70.0),

                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(8.3),
                        40.0,
                        85.0,
                        70.0),

                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(10),
                        36.0,
                        85.0,
                        70.0),

                // new ShootingInterpolator.DistanceAngleSpeedEntry(
                // Units.feetToMeters(12),
                // 34.0,
                // 85.0,
                // 70.0),

                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(13),
                        31.5,
                        85.0,
                        70.0),

                new ShootingInterpolator.DistanceAngleSpeedEntry(
                        Units.feetToMeters(15),
                        27.875,
                        85.0,
                        70.0)

        // // Empirically gathered 15ft shot (prototype)
        // new ShootingInterpolator.DistanceAngleSpeedEntry(
        // Units.feetToMeters(15),
        // 31.0,
        // 85.0,
        // 75.0),
        // // Empirically gathered 21ft shot (prototype)
        // new ShootingInterpolator.DistanceAngleSpeedEntry(
        // Units.feetToMeters(21),
        // 26.0,
        // 85.0,
        // 70.0)
        );
    }

    public static void updateRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public static Pose2d getRobotPose() {
        return robotPose;
    }

    public static void updateVisibleAprilTags(Set<TargetWithSource> trackedAprilTags) {
        visibleAprilTags = trackedAprilTags;
    }

    public static Set<TargetWithSource> getVisibleAprilTags() {
        return visibleAprilTags;
    }

    public static ShootingInterpolator.InterpolatedCalculation getShootingCalculation() {
        return shootingCalculation;
    }

    @Override
    public void periodic() {
        double distanceFromSpeaker = OffsetTags.SPEAKER_AIM.getDistanceFrom(robotPose);
        shootingCalculation = shootingInterpolator.calculateInterpolation(distanceFromSpeaker);

        {
            String calcLogRoot = logRoot + "ShootingCalculation/";
            Logger.recordOutput(calcLogRoot + "DistanceMeters", distanceFromSpeaker);
            Logger.recordOutput(calcLogRoot + "DistanceFeet", Units.metersToFeet(distanceFromSpeaker));
            Logger.recordOutput(calcLogRoot + "AngleDegrees", shootingCalculation.angleDegrees());
            Logger.recordOutput(calcLogRoot + "LeftSpeedRotPerSec", shootingCalculation.leftSpeedRotPerSec());
            Logger.recordOutput(calcLogRoot + "RightSpeedRotPerSec", shootingCalculation.rightSpeedRotPerSec());
        }

        {
            String calcLogRoot = logRoot + "VisionCalculations/";
            Logger.recordOutput(calcLogRoot + "AprilTags", visibleAprilTags.stream()
                    .map((TargetWithSource source) -> source.target().getFiducialId())
                    .collect(Collectors.toList())
                    .stream()
                    .mapToInt(Integer::intValue)
                    .distinct()
                    .sorted()
                    .toArray());
        }
    }

    public static boolean isElevatorUp() {
        return BobotState.isElevatorUp;
    }

    public static boolean isElevatorDown() {
        return !BobotState.isElevatorUp;
    }

    public static void setElevatorUp(boolean isElevatorUp) {
        BobotState.isElevatorUp = isElevatorUp;
    }

    @Override
    public void simulationPeriodic() {
    }
}
