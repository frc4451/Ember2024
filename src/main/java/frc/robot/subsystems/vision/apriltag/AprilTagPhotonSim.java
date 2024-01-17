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

    private PhotonCameraSim cameraSim;

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
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        PhotonPipelineResult frame = camera.getLatestResult();

        AprilTagFiltering.removeTooFarTargets(frame);
        inputs.frame = frame;
    }

    @Override
    public Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult frame) {
        Optional<EstimatedRobotPose> estimatedPose = AprilTagAlgorithms.estimateRobotPose(frame, estimator);

        VisionConstants.VISION_SYSTEM_SIM.ifPresent((visionSystemSim) -> {
            estimatedPose.ifPresentOrElse(
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

        return estimatedPose;
    }
}
