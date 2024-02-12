package frc.robot.subsystems.vision.apriltag;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.VisionConstants;
import frc.utils.GarageUtils;

public enum StageTags {
    HUMAN(VisionConstants.RED_STAGE_HUMAN, VisionConstants.BLUE_STAGE_HUMAN),
    AMP(VisionConstants.RED_STAGE_AMP, VisionConstants.BLUE_STAGE_AMP),
    CENTER(VisionConstants.RED_STAGE_CENTER, VisionConstants.BLUE_STAGE_CENTER);

    private static final double kPoseOffset = 1.0;
    private final int redId;
    private final int blueId;

    private StageTags(int red, int blue) {
        this.redId = red;
        this.blueId = blue;
    }

    public Pose3d getFieldPose() {
        return GarageUtils.isRedAlliance() ? getRedFieldPose() : getBlueFieldPose();
    }

    public Pose3d getRedFieldPose() {
        return VisionConstants.FIELD_LAYOUT.getTagPose(redId).get();
    }

    public Pose3d getBlueFieldPose() {
        return VisionConstants.FIELD_LAYOUT.getTagPose(blueId).get();
    }

    public Pose2d getPathfindingPose() {
        return GarageUtils.isRedAlliance() ? getRedPathfindingPose() : getBluePathfindingPose();
    }

    public Pose2d getRedPathfindingPose() {
        Pose2d pose = VisionConstants.FIELD_LAYOUT.getTagPose(redId).get().toPose2d();
        return new Pose2d(
                pose.getTranslation().plus(
                        new Translation2d(
                                kPoseOffset * pose.getRotation().getCos(),
                                kPoseOffset * pose.getRotation().getSin())),
                pose.getRotation());
    }

    public Pose2d getBluePathfindingPose() {
        Pose2d pose = VisionConstants.FIELD_LAYOUT.getTagPose(blueId).get().toPose2d();
        return new Pose2d(
                pose.getTranslation().plus(
                        new Translation2d(
                                kPoseOffset * pose.getRotation().getCos(),
                                kPoseOffset * pose.getRotation().getSin())),
                pose.getRotation());
    }

    public Command getDeferredCommand() {
        return Commands.deferredProxy(
                () -> AutoBuilder.pathfindToPose(
                        getPathfindingPose(),
                        PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                        0.0,
                        0.0));
    }

    public int getId() {
        return GarageUtils.isRedAlliance() ? redId : blueId;
    }

    public int getBlueId() {
        return blueId;
    }

    public int getRedId() {
        return redId;
    }
}
