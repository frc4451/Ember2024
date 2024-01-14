package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.VisionConstants;
import frc.robot.VisionConstants.VisionSource;

public class AprilTagPhoton implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final DuplicateTracker dupeTracker = new DuplicateTracker();

    public AprilTagPhoton(VisionSource source) {
        camera = new PhotonCamera(source.name());

        estimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                source.robotToCamera());

        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @AutoLogOutput
    private PhotonPipelineResult frame = new PhotonPipelineResult();

    private void setNoEstimateInputs(AprilTagIOInputs inputs) {
        inputs.estimatedPose = null;
        inputs.estimatedPoseTimestamp = -1.0;
        // inputs.estimatedPoseTargetsUsed = new PhotonTrackedTarget[] {};
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        frame = camera.getLatestResult();

        AprilTagFiltering.removeTooFarTargets(frame);

        inputs.frame = frame;

        // Check if our frame contains duplicates or targets are invalid
        if (dupeTracker.isDuplicateFrame(frame)
                || AprilTagFiltering.shouldIgnoreFrame(frame, AprilTagFiltering.getAllowedIDs())) {
            setNoEstimateInputs(inputs);
        } else {
            Optional<EstimatedRobotPose> possibleRobotPose = estimator.update(frame);
            possibleRobotPose.ifPresentOrElse((estimatedRobotPose) -> {
                inputs.estimatedPose = estimatedRobotPose.estimatedPose;
                inputs.estimatedPoseTimestamp = estimatedRobotPose.timestampSeconds;
                // inputs.estimatedPoseTargetsUsed =
                // estimatedRobotPose.targetsUsed.toArray(PhotonTrackedTarget[]::new);
            }, () -> {
                setNoEstimateInputs(inputs);
            });
        }
    }
}
