package frc.robot.pathplanner.paths;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.VisionConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.utils.GarageUtils;

public enum PathPlannerPoses {
    SPEAKER_CENTER(
            "SpeakerCenter",
            new Pose2d(14.33, 5.5, Rotation2d.fromDegrees(0)),
            new Pose2d(1.83, 5.5, Rotation2d.fromDegrees(180))),
    SPEAKER_LEFT(
            "SpeakerLeft",
            new Pose2d(14.33, 6.0, Rotation2d.fromDegrees(-30)),
            new Pose2d(1.83, 5.0, Rotation2d.fromDegrees(150))),
    SPEAKER_RIGHT(
            "SpeakerRight",
            new Pose2d(14.33, 5.0, Rotation2d.fromDegrees(30)),
            new Pose2d(1.83, 6.0, Rotation2d.fromDegrees(-150))),
    AMP(
            "Amp",
            new Pose2d(14.70, 8.0, Rotation2d.fromDegrees(90)),
            new Pose2d(1.84, 8.0, Rotation2d.fromDegrees(90))),
    OTHER_AMP(
            "Other Amp",
            new Pose2d(1.84, 8.0, Rotation2d.fromDegrees(90)),
            new Pose2d(14.70, 8.0, Rotation2d.fromDegrees(90))),
    HUMAN_PLAYER(
            "Human Player",
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(-120)),
            new Pose2d(15.5, 1.25, Rotation2d.fromDegrees(-60))),
    STAGE_CENTER(
            "Stage Center",
            VisionConstants.FIELD_LAYOUT.getTagPose(VisionConstants.RED_STAGE_CENTER).get().toPose2d()
                    .plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d())),
            VisionConstants.FIELD_LAYOUT.getTagPose(VisionConstants.BLUE_STAGE_CENTER).get().toPose2d()
                    .plus(new Transform2d(new Translation2d(0.5, 0), new Rotation2d()))),
    STAGE_HUMAN(
            "Stage Human",
            VisionConstants.FIELD_LAYOUT.getTagPose(VisionConstants.RED_STAGE_HUMAN).get().toPose2d()
                    .plus(new Transform2d(new Translation2d(0.5, -0.25), new Rotation2d())),
            VisionConstants.FIELD_LAYOUT.getTagPose(VisionConstants.BLUE_STAGE_HUMAN).get().toPose2d()
                    .plus(new Transform2d(new Translation2d(0.25, 0.25), new Rotation2d()))),
    STAGE_AMP(
            "Stage Amp",
            VisionConstants.FIELD_LAYOUT.getTagPose(VisionConstants.RED_STAGE_AMP).get().toPose2d()
                    .plus(new Transform2d(new Translation2d(-0.25, -0.25), new Rotation2d())),
            VisionConstants.FIELD_LAYOUT.getTagPose(VisionConstants.BLUE_STAGE_AMP).get().toPose2d()
                    .plus(new Transform2d(new Translation2d(0.25, 0.25), new Rotation2d())))

    //
    ;

    public final String label;

    public final Pose2d redPose;

    public final Pose2d bluePose;

    /**
     * @param label   Label that is mainly used for Shuffleboard
     * @param routine Function (or reference) that will be called periodically by
     *                this AutoState
     * @param paths   List of PathPlannerTrajectory's that is usually generated with
     *                {@code PathPlanner.loadPathGroup()}
     */
    private PathPlannerPoses(String label, Pose2d redPose, Pose2d bluePose) {
        this.label = label;
        this.redPose = redPose;
        this.bluePose = bluePose;
    }

    public Command getDeferredCommand() {
        return Commands.deferredProxy(
                () -> AutoBuilder.pathfindToPose(
                        GarageUtils.isRedAlliance()
                                ? redPose
                                : bluePose,
                        PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                        0.0,
                        0.0));
    }
}
