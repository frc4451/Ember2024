package frc.robot.bobot_state.TargetAngleTrackers;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;
import frc.robot.subsystems.vision.apriltag.OffsetTags;
import frc.utils.GarageUtils;

public class SpeakerAngleTracker extends TargetAngleTracker {
    private boolean hasSeenTag = false;
    private OffsetTags tag = OffsetTags.SPEAKER_AIM;
    private Pose3d targetPose = tag.getPose();
    private Optional<Rotation2d> rotationTarget;

    public SpeakerAngleTracker() {
        super();
    }

    public boolean getHasSeenTag() {
        return hasSeenTag;
    }

    public Pose3d getTargetPose() {
        return targetPose;
    }

    public Optional<Rotation2d> getRotationTarget() {
        return rotationTarget;
    }

    public void update() {
        Pose3d robotPose = new Pose3d(BobotState.getRobotPose());

        Set<TargetWithSource> targets = BobotState.getVisibleAprilTags();
        AprilTagAlgorithms.filterTags(targets.stream(), GarageUtils.getSpeakerTag())
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity()
                                        ? targetWithSourceA
                                        : targetWithSourceB)
                .ifPresent(
                        targetWithSource -> {
                            this.hasSeenTag = true;
                            this.targetPose = targetWithSource.getTargetPoseFrom(robotPose);
                        });
        rotationTarget = Optional.of(
                targetPose
                        .relativeTo(robotPose)
                        .getTranslation()
                        .toTranslation2d()
                        .getAngle()
                        .plus(BobotState.getRobotPose().getRotation())
                        .plus(new Rotation2d(Math.PI)));
    }
}
