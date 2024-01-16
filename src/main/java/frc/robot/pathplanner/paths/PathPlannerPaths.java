package frc.robot.pathplanner.paths;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathPlannerConstants;
import frc.utils.GarageUtils;

public enum PathPlannerPaths {
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

    HUMAN_PLAYER(
            "HumanPlayer",
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(-120)),
            new Pose2d(15.5, 1.25, Rotation2d.fromDegrees(-60)));

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
    private PathPlannerPaths(String label, Pose2d redPose, Pose2d bluePose) {
        this.label = label;
        this.redPose = redPose;
        this.bluePose = bluePose;
    }

    public Command getCommand() {
        Pose2d target = GarageUtils.getAlliance() == Alliance.Red
                ? redPose
                : bluePose;

        return AutoBuilder.pathfindToPose(
                target,
                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                0.0,
                0.0);
    }
}
