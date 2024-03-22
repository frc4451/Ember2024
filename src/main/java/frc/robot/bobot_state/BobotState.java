package frc.robot.bobot_state;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.bobot_state.TargetAngleTrackers.NoteAngleTracker;
import frc.robot.bobot_state.TargetAngleTrackers.SpeakerAngleTracker;
import frc.robot.subsystems.pivot.PivotLocation;
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

    private static Optional<PhotonTrackedTarget> closestObject = Optional.empty();

    private static boolean isElevatorUp = false;

    private static AimingMode aimingMode = AimingMode.NONE;

    private static final SpeakerAngleTracker speakerAngleTracker = new SpeakerAngleTracker();
    private static final NoteAngleTracker noteAngleTracker = new NoteAngleTracker();

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
                        PivotLocation.kElevatorDownSoftMax.angle.getDegrees(),
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

    public static Pose3d getRobotPose3d() {
        return new Pose3d(BobotState.getRobotPose());
    }

    public static void updateVisibleAprilTags(Set<TargetWithSource> trackedAprilTags) {
        visibleAprilTags = trackedAprilTags;
    }

    public static Set<TargetWithSource> getVisibleAprilTags() {
        return visibleAprilTags;
    }

    public static void updateClosestObject(Optional<PhotonTrackedTarget> target) {
        closestObject = target;
    }

    public static Optional<PhotonTrackedTarget> getClosestObject() {
        return closestObject;
    }

    public static ShootingInterpolator.InterpolatedCalculation getShootingCalculation() {
        return shootingCalculation;
    }

    public static void updateAimingMode(AimingMode newAimingMode) {
        aimingMode = newAimingMode;
    }

    public static AimingMode getAimingMode() {
        return aimingMode;
    }

    public static SpeakerAngleTracker getSpeakerAngleTracker() {
        return speakerAngleTracker;
    }

    public static NoteAngleTracker getNoteAngleTracker() {
        return noteAngleTracker;
    }

    /**
     * Vision Assisted Rotation Correction (VARC) for PathPlanner rotation overrides
     *
     * https://pathplanner.dev/pplib-override-target-rotation.html
     */
    public static Optional<Rotation2d> VARC() {
        return switch (BobotState.getAimingMode()) {
            case OBJECT_DETECTION -> BobotState.getNoteAngleTracker().getRotationTarget();
            case SPEAKER -> BobotState.getSpeakerAngleTracker().getRotationTarget();
            case NONE -> Optional.empty();
            default -> Optional.empty();
        };
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

        {
            speakerAngleTracker.update();
            noteAngleTracker.update();
            Logger.recordOutput(logRoot + "AimingMode", BobotState.getAimingMode());

            {
                String calcRoot = logRoot + "AngleTracking/Speaker/";
                Logger.recordOutput(calcRoot + "TargetPose", speakerAngleTracker.getTargetPose());
                Logger.recordOutput(calcRoot + "TargetAngleRad",
                        speakerAngleTracker.getRotationTarget()
                                .map(Rotation2d::getRadians)
                                .orElse(Double.NaN));
                Logger.recordOutput(calcRoot + "TargetAngleDegrees",
                        speakerAngleTracker.getRotationTarget()
                                .map(Rotation2d::getDegrees)
                                .orElse(Double.NaN));
                Logger.recordOutput(calcRoot + "HasSeenTag", speakerAngleTracker.getHasSeenTag());
            }

            {
                String calcRoot = logRoot + "AngleTracking/Note/";
                Logger.recordOutput(calcRoot + "TargetAngleRad",
                        noteAngleTracker.getRotationTarget()
                                .map(Rotation2d::getRadians)
                                .orElse(Double.NaN));
                Logger.recordOutput(calcRoot + "TargetAngleDegrees",
                        noteAngleTracker.getRotationTarget()
                                .map(Rotation2d::getRadians)
                                .orElse(Double.NaN));
                Logger.recordOutput(calcRoot + "HasSeenNote", noteAngleTracker.getHasSeenNote());
            }
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
