package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.apriltag.StageTags;

/**
 * Utilities methods for solving common Geometric problems
 */
public class GeomUtils {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Takes a Pose2d and returns a inverted version of itself
     *
     * @param pose The pose that we want to inverse
     * @return resulting Pose2d
     */
    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
                pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Using the robot's known pose, find the distance of how far away the robot
     * is from the `StageTag`.
     *
     * @param tag       - Stage Tag to aim
     * @param robotPose - Current Robot Pose
     * @return distance from robot to target
     */
    public static double getDistanceFromTag(Pose2d robotPose, StageTags tag) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation = tag.getPose().toPose2d().getTranslation();
        double distanceToTarget = robotTranslation.getDistance(targetTranslation);
        return distanceToTarget;
    }

}
