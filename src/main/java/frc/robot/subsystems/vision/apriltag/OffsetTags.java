package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.VisionConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.utils.GarageUtils;

public enum OffsetTags {
    STAGE_HUMAN(VisionConstants.RED_STAGE_HUMAN, VisionConstants.BLUE_STAGE_HUMAN, 1.0, new Rotation3d(0, 0, Math.PI)),
    STAGE_AMP(VisionConstants.RED_STAGE_AMP, VisionConstants.BLUE_STAGE_AMP, 1.0, new Rotation3d(0, 0, Math.PI)),
    STAGE_CENTER(
            VisionConstants.RED_STAGE_CENTER,
            VisionConstants.BLUE_STAGE_CENTER,
            1.0,
            new Rotation3d(0, 0, Math.PI)),
    SPEAKER_AIM(VisionConstants.RED_SPEAKER_CENTER, VisionConstants.BLUE_SPEAKER_CENTER, 0.0),
    SPEAKER_10FT(VisionConstants.RED_SPEAKER_CENTER, VisionConstants.BLUE_SPEAKER_CENTER, Units.feetToMeters(10)),
    SPEAKER_15FT(VisionConstants.RED_SPEAKER_CENTER, VisionConstants.BLUE_SPEAKER_CENTER, Units.feetToMeters(15)),
    AMP(VisionConstants.RED_AMP_TAG, VisionConstants.BLUE_AMP_TAG, 1.0, new Rotation3d(0, 0, Math.PI)),
    OTHER_AMP(VisionConstants.BLUE_AMP_TAG, VisionConstants.RED_AMP_TAG, 1.0, new Rotation3d(0, 0, Math.PI)),
    HUMAN_PLAYER(
            VisionConstants.RED_HUMAN_PLAYER_INSIDE,
            VisionConstants.BLUE_HUMAN_PLAYER_INSIDE,
            1.0,
            new Rotation3d(0, 0, Math.PI)),
    FLOOR_SHOT(VisionConstants.RED_AMP_TAG, VisionConstants.BLUE_AMP_TAG, 2.0);

    private final int redId;
    private final int blueId;
    private final double poseOffsetMeters;
    private final Rotation3d extraRotation;

    private OffsetTags(int red, int blue, double poseOffsetMeters) {
        this(red, blue, poseOffsetMeters, new Rotation3d());
    }

    private OffsetTags(int red, int blue, double poseOffsetMeters, Rotation3d extraRotation) {
        this.redId = red;
        this.blueId = blue;
        this.poseOffsetMeters = poseOffsetMeters;
        this.extraRotation = extraRotation;
    }

    public Pose3d getOffsetPoseFrom(Pose3d pose) {
        return new Pose3d(
                pose.getTranslation().plus(
                        new Translation3d(
                                poseOffsetMeters * Math.cos(pose.getRotation().getZ()),
                                poseOffsetMeters * Math.sin(pose.getRotation().getZ()),
                                0)),
                pose.getRotation().plus(extraRotation));
    }

    public Pose3d getOffsetPose() {
        return GarageUtils.isRedAlliance() ? getRedOffsetPose() : getBlueOffsetPose();
    }

    public Pose3d getRedOffsetPose() {
        Pose3d pose = VisionConstants.FIELD_LAYOUT.getTagPose(redId).get();
        return getOffsetPoseFrom(pose);
    }

    public Pose3d getBlueOffsetPose() {
        Pose3d pose = VisionConstants.FIELD_LAYOUT.getTagPose(blueId).get();
        return getOffsetPoseFrom(pose);
    }

    public Command getDeferredCommand() {
        return Commands.deferredProxy(
                () -> AutoBuilder.pathfindToPose(
                        getOffsetPose().toPose2d(),
                        PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                        0.0,
                        0.0));
    }

    public Pose3d getPose() {
        return VisionConstants.FIELD_LAYOUT.getTagPose(getId()).get();
    }

    public Pose3d getRedPose() {
        return VisionConstants.FIELD_LAYOUT.getTagPose(getRedId()).get();
    }

    public Pose3d getBluePose() {
        return VisionConstants.FIELD_LAYOUT.getTagPose(getBlueId()).get();
    }

    public int getId() {
        return GarageUtils.isRedAlliance() ? redId : blueId;
    }

    public int getRedId() {
        return redId;
    }

    public int getBlueId() {
        return blueId;
    }

    /**
     * Using the robot's known pose, find the distance of how far away the robot is
     * from the {@link #OffsetTags}.
     *
     * @param pose - Current Robot Pose
     * @return Distance from robot to target (meters)
     */
    public double getFieldDistanceFrom(Pose2d pose) {
        Translation2d poseTranslation = pose.getTranslation();
        Translation2d targetTranslation = getPose().toPose2d().getTranslation();
        double distanceToTarget = poseTranslation.getDistance(targetTranslation);

        return distanceToTarget;
    }

    /**
     * Using the robot's known pose, find the distance of how far away the robot is
     * from the {@link #OffsetTags}.
     *
     * @param pose - Current Robot Pose
     * @return Distance from robot to target (meters)
     */
    public Optional<Double> getVisionDistanceFrom(Pose2d pose) {
        Optional<Double> distance = Optional.empty();

        Optional<TargetWithSource> targetWithSource = AprilTagAlgorithms
                .filterTags(BobotState.getVisibleAprilTags().stream(), getId())
                .reduce((targetWithSourceA,
                        targetWithSourceB) -> targetWithSourceA.target().getPoseAmbiguity() <= targetWithSourceB
                                .target().getPoseAmbiguity()
                                        ? targetWithSourceA
                                        : targetWithSourceB);

        if (targetWithSource.isPresent()) {
            Translation2d poseTranslation = pose.getTranslation();

            Pose3d tagPose = targetWithSource.get().getTargetPoseFrom(new Pose3d(pose));
            Pose3d targetPose = getOffsetPoseFrom(tagPose);
            Translation2d targetTranslation = targetPose.toPose2d().getTranslation();
            distance = Optional.of(poseTranslation.getDistance(targetTranslation));
        }

        return distance;
    }

    /**
     * Using the robot's known pose, find the distance of how far away the robot is
     * from the {@link #OffsetTags}.
     *
     * @param pose - Current Robot Pose
     * @return Distance from robot to target (meters)
     */
    public double getDistanceFrom(Pose2d pose) {
        return getVisionDistanceFrom(pose).orElse(getFieldDistanceFrom(pose));
    }
}
