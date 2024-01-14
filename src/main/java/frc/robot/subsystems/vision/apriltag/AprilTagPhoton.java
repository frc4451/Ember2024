package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.VisionConstants;
import frc.robot.VisionConstants.VisionSource;

public class AprilTagPhoton implements AprilTagIO {
    private final AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final DuplicateTracker dupeTracker = new DuplicateTracker();

    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

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
    public void updateInputs() {
        PhotonPipelineResult frame = camera.getLatestResult();
        AprilTagFiltering.removeTooFarTargets(frame);
        inputs.frame = frame;
    }

    @Override
    public void periodic() {
        // Check if our frame contains duplicates or targets are invalid
        if (dupeTracker.isDuplicateFrame(inputs.frame)
                || AprilTagFiltering.shouldIgnoreFrame(inputs.frame, AprilTagFiltering.getAllowedIDs())) {
            estimatedPose = Optional.empty();
        } else {
            estimatedPose = estimator.update(inputs.frame);
        }

        Logger.processInputs("AprilTagCam/" + camera.getName(), inputs);
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return estimatedPose;
    }
}
