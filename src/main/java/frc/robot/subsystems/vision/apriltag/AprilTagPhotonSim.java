package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.VisionSource;

public class AprilTagPhotonSim implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final DuplicateTracker duplicateTracker = new DuplicateTracker();

    private PhotonCameraSim cameraSim;

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

    public AprilTagPhotonSim(VisionSource source) {
        camera = new PhotonCamera(source.name());

        estimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                source.robotToCamera());

        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        VisionConstants.VISION_SYSTEM_SIM.ifPresent((visionSim) -> {
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);

            SimCameraProperties simCameraProperties = new SimCameraProperties();

            // All of our AprilTag cameras use 16:9 FHD resolution
            simCameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
            // I read this from the docs, but this may need adjusting
            simCameraProperties.setCalibError(0.25, 0.08);
            simCameraProperties.setFPS(20.0);
            simCameraProperties.setAvgLatencyMs(35);
            simCameraProperties.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, simCameraProperties);

            visionSim.addCamera(cameraSim, source.robotToCamera());
            cameraSim.enableDrawWireframe(true);
            cameraSim.setMaxSightRange(10.0);
            cameraSim.setWireframeResolution(1);
        });

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
        VisionConstants.VISION_SYSTEM_SIM.ifPresent((visionSystemSim) -> {
            estimatedRobotPose.ifPresentOrElse(
                    (est) -> {
                        visionSystemSim
                                .getDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d());
                    },
                    () -> {
                        visionSystemSim
                                .getDebugField()
                                .getObject("VisionEstimation")
                                .setPoses();
                    });
        });

        return estimatedRobotPose;
    }
}
