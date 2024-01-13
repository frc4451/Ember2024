package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.VisionConstants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagPhoton;
import frc.robot.subsystems.vision.apriltag.AprilTagIO.AprilTagIOInputs;
import frc.utils.VirtualSubsystem;

public class VisionSubsystem extends VirtualSubsystem {
    public static record VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {
    }

    public static record AprilTagCam(AprilTagIO io, AprilTagIOInputs inputs) {
    }

    private final List<AprilTagCam> aprilTagCameras = new ArrayList<>();

    private ConcurrentLinkedQueue<VisionMeasurement> visionMeasurements = new ConcurrentLinkedQueue<>();

    public VisionSubsystem() {
        // Initialize all cameras that we have pre-configured from VisionConstants.
        //
        // Using the VISION_SOURCES constants, we read each camera established in the
        // Constants file and populate our camera_estimators available.
        for (VisionConstants.VisionSource source : VisionConstants.APRIL_TAG_SOURCES) {
            AprilTagIO io;

            switch (AdvantageKitConstants.getMode()) {
                case REAL:
                    io = new AprilTagPhoton(source);
                    break;
                // case SIM:
                // io = new AprilTagPhoton(source);
                // break;
                case REPLAY:
                default:
                    io = new AprilTagIO() {
                    };
                    break;
            }
            aprilTagCameras.add(new AprilTagCam(io, new AprilTagIOInputs()));
        }

    }

    // Enforce periodic method for VirtualSubsystem
    @Override
    public void periodic() {
        // Check for updates to Measurements
        if (VisionConstants.FIELD_LAYOUT != null) {
            this.findVisionMeasurements();
        }
    }

    /**
     * Using all available camera estimators, we poll each and try to build
     * our VisionMeasurements to help correct our position and odometry.
     */
    private void findVisionMeasurements() {
        // For each camera we need to do the following:
        for (AprilTagCam cam : aprilTagCameras) {
            cam.io.updateInputs(cam.inputs);
            // Add our estimated position and deviation to be used by our
            // SwerveDrivePoseEstimator
            // visionMeasurements.add(new VisionMeasurement(estimatedPose, confidence));
        }
    }

    // Return the head of the Queue as a single measurement
    public VisionMeasurement pollLatestVisionMeasurement() {
        return visionMeasurements.poll();
    }

    /**
     * Gets a list of the Fiducial IDs for April Tags we detect
     *
     * @return Pipe concatonated list of IDs
     */
    // public String getVisibleTargets() {
    // List<Integer> targetIds = new ArrayList<>();

    // for (CameraEstimator cameraEstimator : aprilTagInputs) {
    // PhotonPipelineResult result = cameraEstimator.camera().getLatestResult();
    // for (PhotonTrackedTarget target : result.targets) {
    // targetIds.add(target.getFiducialId());
    // }
    // }

    // Set<Integer> uniqueIds = new HashSet<>(targetIds);
    // List<Integer> organizedIds = new ArrayList<>(uniqueIds);

    // Collections.sort(organizedIds);

    // String visibleTargets = organizedIds.stream()
    // .map(Object::toString)
    // .reduce((s1, s2) -> s1 + " | " + s2)
    // .orElse("");

    // return visibleTargets;
    // }
}
