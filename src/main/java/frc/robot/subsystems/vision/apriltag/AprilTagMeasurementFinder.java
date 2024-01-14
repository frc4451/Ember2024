package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;
import frc.robot.subsystems.vision.apriltag.AprilTagIO.AprilTagIOInputs;

public class AprilTagMeasurementFinder {
    /**
     * Create a {@link VisionMeasurement} with a pose & confidence value from an
     * {@link EstimatedRobotPose}
     */
    public static Optional<VisionMeasurement> findVisionMeasurement(AprilTagIOInputs inputs) {
        // For ease of porting from old system do this for now
        EstimatedRobotPose estimatedPose = new EstimatedRobotPose(
                inputs.estimatedPose,
                inputs.estimatedPoseTimestamp,
                // Arrays.asList(inputs.estimatedPoseTargetsUsed),
                new ArrayList<PhotonTrackedTarget>(),
                null); // The strategy used doesn't actually matter here so let's go with null

        // Check if we only have one target in sight, and check that it's clear enough
        // to read.
        if (estimatedPose.targetsUsed.size() == 1
                && (estimatedPose.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.POSE_AMBIGUITY_CUTOFF
                        || estimatedPose.targetsUsed.get(0).getPoseAmbiguity() == -1)) {
            return Optional.empty();
        }

        // Calculates the sums of every distance using Euclidiean Norm.
        // Think of it as the Pythagorean theorem except in three dimensions.
        double sumDistance = estimatedPose.targetsUsed.stream()
                .mapToDouble(
                        target -> {
                            Transform3d target_position = target.getBestCameraToTarget();
                            return Math.sqrt(
                                    Math.pow(target_position.getX(), 2)
                                            + Math.pow(target_position.getY(), 2)
                                            + Math.pow(target_position.getZ(), 2));
                        })
                .sum();

        double averageDistance = sumDistance / estimatedPose.targetsUsed.size();

        // Calculate our 3x1 confidence matrix by clamping the number of targets
        // against the number of targets we see vs the number of targets we
        // can reliably calculate against.
        Matrix<N3, N1> confidence = VisionConstants.TAG_COUNT_DEVIATION_PARAMS
                .get(
                        MathUtil.clamp(
                                estimatedPose.targetsUsed.size() - 1,
                                0,
                                VisionConstants.TAG_COUNT_DEVIATION_PARAMS.size() - 1))
                .computeDeviation(averageDistance);

        return Optional.of(new VisionMeasurement(estimatedPose, confidence));
    }
}
