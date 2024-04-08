package frc.robot.bobot_state;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.ShootingInterpolator.InterpolatedCalculation;
import frc.robot.bobot_state.TargetAngleTrackers.NoteAngleTracker;
import frc.robot.bobot_state.TargetAngleTrackers.SpeakerAngleTracker;
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

    private static final InterpolationTriplet speakerTriplet = new InterpolationTriplet(
            "Speaker",
            new SpeakerInterpolator());

    private static final InterpolationTriplet floorTriplet = new InterpolationTriplet(
            "Floor",
            new FloorInterpolator());

    private static final List<InterpolationTriplet> interpolationTriplets = List.of(speakerTriplet, floorTriplet);

    public static final double kLeftShooterSpeed = 88.0;

    public static final double kRightShooterSpeed = 73.0;

    private static Pose2d robotPose = new Pose2d();

    /**
     * {@link #robotPose} predicted ahead via a pose expontential of our current
     * velocity
     */
    private static Pose2d predictedPose = new Pose2d();

    private static Set<TargetWithSource> visibleAprilTags = new HashSet<>();

    private static Optional<PhotonTrackedTarget> closestObject = Optional.empty();

    private static boolean isElevatorUp = false;

    private static AimingMode aimingMode = AimingMode.NONE;

    private static final SpeakerAngleTracker speakerAngleTracker = new SpeakerAngleTracker();
    private static final NoteAngleTracker noteAngleTracker = new NoteAngleTracker();

    public static Trigger inRangeOfSpeakerInterpolation() {
        return new Trigger(() -> OffsetTags.SPEAKER_AIM.getDistanceFrom(robotPose) < Units.feetToMeters(15));
    }

    public static void updateRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public static Pose2d getRobotPose() {
        return robotPose;
    }

    public static void updatePredictedPose(Pose2d pose) {
        predictedPose = pose;
    }

    public static Pose2d getPredictedPose() {
        return predictedPose;
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

    public static InterpolatedCalculation getSpeakerCalculation() {
        return speakerTriplet.calculation;
    }

    public static InterpolatedCalculation getFloorCalculation() {
        return floorTriplet.calculation;
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
        {
            String calcLogRoot = logRoot + "RobotOdometry/";
            Logger.recordOutput(calcLogRoot + "Estimated", robotPose);
            Logger.recordOutput(calcLogRoot + "Predicted", predictedPose);
        }

        interpolationTriplets.forEach((InterpolationTriplet triplet) -> {
            triplet.interpolator.update(robotPose);

            double distanceFromSpeaker = triplet.interpolator.getDistanceFromTarget();
            triplet.calculation = triplet.interpolator.calculateInterpolation();

            String calcLogRoot = logRoot + "Interpolators/" + triplet.name + "/";
            Logger.recordOutput(calcLogRoot + "DistanceMeters", distanceFromSpeaker);
            Logger.recordOutput(calcLogRoot + "DistanceFeet", Units.metersToFeet(distanceFromSpeaker));
            Logger.recordOutput(calcLogRoot + "AngleDegrees", triplet.calculation.angleDegrees());
            Logger.recordOutput(calcLogRoot + "LeftSpeedRotPerSec", triplet.calculation.leftSpeedRotPerSec());
            Logger.recordOutput(calcLogRoot + "RightSpeedRotPerSec", triplet.calculation.rightSpeedRotPerSec());
        });

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
