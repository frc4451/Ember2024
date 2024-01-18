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
    private final DuplicateTracker duplicateTracker = new DuplicateTracker();

    private final Thread periodicThread = new Thread(() -> {
        while (!Thread.currentThread().isInterrupted()) {
            periodic();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    });

    public AprilTagPhoton(VisionSource source) {
        camera = new PhotonCamera(source.name());

        estimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                source.robotToCamera());

        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        periodicThread.setPriority(Thread.MAX_PRIORITY);
        periodicThread.start();
    }

    private PhotonPipelineResult frame = new PhotonPipelineResult();
    private boolean isDuplicateFrame = false;
    private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    private void periodic() {
        PhotonPipelineResult frame = camera.getLatestResult();

        isDuplicateFrame = duplicateTracker.isDuplicateFrame(frame);

        if (isDuplicateFrame) {
            return;
        }

        AprilTagFiltering.removeTooFarTargets(frame);
        this.frame = frame;
        estimatedRobotPose = AprilTagAlgorithms.estimateRobotPose(frame, estimator);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        inputs.frame = frame;
        inputs.isDuplicateFrame = isDuplicateFrame;
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return estimatedRobotPose;
    }
}
