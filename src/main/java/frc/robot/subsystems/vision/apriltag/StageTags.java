package frc.robot.subsystems.vision.apriltag;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.VisionConstants;
import frc.utils.GarageUtils;

public enum StageTags {
    HUMAN(VisionConstants.RED_STAGE_HUMAN, VisionConstants.BLUE_STAGE_HUMAN, 1.0),
    AMP(VisionConstants.RED_STAGE_AMP, VisionConstants.BLUE_STAGE_AMP, 1.0),
    CENTER(VisionConstants.RED_STAGE_CENTER, VisionConstants.BLUE_STAGE_CENTER, 1.0),
    SPEAKER_AIM(VisionConstants.RED_SPEAKER_CENTER, VisionConstants.BLUE_SPEAKER_CENTER, 0.0),
    SPEAKER_10FT(VisionConstants.RED_SPEAKER_CENTER, VisionConstants.BLUE_SPEAKER_CENTER, Units.feetToMeters(10)),
    SPEAKER_15FT(VisionConstants.RED_SPEAKER_CENTER, VisionConstants.BLUE_SPEAKER_CENTER, Units.feetToMeters(15)),
    ;

    private final int redId;
    private final int blueId;
    private final double poseOffsetMeters;

    private StageTags(int red, int blue, double poseOffsetMeters) {
        this.redId = red;
        this.blueId = blue;
        this.poseOffsetMeters = poseOffsetMeters;
    }

    public Pose3d getOffsetPoseFrom(Pose3d pose) {
        return new Pose3d(
                pose.getTranslation().plus(
                        new Translation3d(
                                poseOffsetMeters * Math.cos(pose.getRotation().getZ()),
                                poseOffsetMeters * Math.sin(pose.getRotation().getZ()),
                                0)),
                pose.getRotation());
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
}
