package frc.utils.TargetAngleTrackers;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

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

    public SpeakerAngleTracker() {
        super();
    }

    public boolean getHasSeenTag() {
        return hasSeenTag;
    }

    public Pose3d getTargetPose() {
        return targetPose;
    }

    public Rotation2d getRotationDifference() {
        return targetPose
                .relativeTo(BobotState.getRobotPose3d())
                .getTranslation()
                .toTranslation2d()
                .getAngle()
                .plus(BobotState.getRobotPose().getRotation())
                .plus(new Rotation2d(Math.PI));
    }

    public void update() {
        Pose3d robotPose = BobotState.getRobotPose3d();

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
    }

    public void log() {
        String logRoot = "TargetAngle/Speaker/";
        Logger.recordOutput(logRoot + "TargetPose", this.targetPose);
        Logger.recordOutput(logRoot + "TargetAngleRad", this.getRotationDifference().getRadians());
        Logger.recordOutput(logRoot + "TargetAngleDegrees", this.getRotationDifference().getDegrees());
        Logger.recordOutput(logRoot + "HasSeenTag", this.hasSeenTag);
    }
}
