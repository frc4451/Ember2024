package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.VisionConstants;
import frc.robot.VisionConstants.VisionSource;

public class AprilTagPhoton implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public AprilTagPhoton(VisionSource source) {
        camera = new PhotonCamera(source.name());

        estimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                source.robotToCamera());

        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        PhotonPipelineResult frame = camera.getLatestResult();
        AprilTagFiltering.removeTooFarTargets(frame);
        inputs.frame = frame;
    }

    @Override
    public Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult frame) {
        return AprilTagAlgorithms.estimateRobotPose(frame, estimator);
    }
}
