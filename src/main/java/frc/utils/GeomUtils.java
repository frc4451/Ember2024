// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.VisionConstants;

/**
 * Geometry utilities for working with translations, rotations, transforms, and
 * poses.
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
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Rotation2d rotation) {
        return new Transform2d(new Translation2d(), rotation);
    }

    /**
     * Converts a {@link Pose2d} to a {@link Transform2d} to be used in a kinematic
     * chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Takes a {@link Pose2d} and returns a inverted version of itself
     *
     * @param pose The pose that we want to inverse
     * @return resulting pose
     */
    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
                pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Converts a {@link Transform2d} to a {@link Pose2d} to be used as a position
     * or as the start of a kinematic chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist  The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiply(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start
     * of a kinematic chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z).
     * chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Creates a new pose from an existing one using a different translation value.
     *
     * @param pose        The original pose
     * @param translation The new translation to use
     * @return The new pose with the new translation and original rotation
     */
    public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
        return new Pose2d(translation, pose.getRotation());
    }

    /**
     * Creates a new pose from an existing one using a different rotation value.
     *
     * @param pose     The original pose
     * @param rotation The new rotation to use
     * @return The new pose with the original translation and new rotation
     */
    public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(pose.getTranslation(), rotation);
    }

    /**
     * Assuming that the `PhotonTrackedTarget` provided is a Note, calculate
     * the distance/rotation from the Note and return that
     *
     * @param target - PhotonTrackedTarget from Object Detection Camera
     * @return transform from robot-to-note
     */
    public static Transform2d getTransformFromNote(PhotonTrackedTarget target) {
        Transform3d robotToCamera = VisionConstants.OBJECT_DETECTION_SOURCE.robotToCamera();

        // The distance in a straight line from the camera to the target
        double distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(),
                Units.inchesToMeters(2),
                -robotToCamera.getRotation().getY(),
                Units.degreesToRadians(target.getPitch()));

        Translation2d cameraToTarget = PhotonUtils.estimateCameraToTargetTranslation(
                distanceToTargetMeters,
                Rotation2d.fromDegrees(target.getYaw()));

        // The transform to the target from the robot
        Transform2d robotToTarget = new Transform2d(
                robotToCamera.getTranslation().toTranslation2d().plus(cameraToTarget),
                robotToCamera.getRotation().toRotation2d().plus(cameraToTarget.getAngle()));

        return robotToTarget;
    }
}
