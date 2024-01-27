package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.VisionSource;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagIOInputsAutoLogged;
import frc.robot.subsystems.vision.apriltag.AprilTagPhoton;
import frc.robot.subsystems.vision.apriltag.AprilTagPhotonSim;
import frc.robot.subsystems.vision.apriltag.DuplicateTracker;
import frc.robot.subsystems.vision.apriltag.EstimatedPose;
import frc.robot.subsystems.vision.object_detection.ObjectDetectionFiltering;
import frc.robot.subsystems.vision.object_detection.ObjectDetectionIO;
import frc.robot.subsystems.vision.object_detection.ObjectDetectionIOInputsAutoLogged;
import frc.robot.subsystems.vision.object_detection.ObjectDetectionPhoton;
import frc.robot.subsystems.vision.object_detection.ObjectDetectionPhotonSim;
import frc.utils.VirtualSubsystem;

public class VisionSubsystem extends VirtualSubsystem {
    public static record VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {
    }

    public static record AprilTagCamera(
            AprilTagIO io,
            AprilTagIOInputsAutoLogged inputs,
            VisionSource source,
            DuplicateTracker dupeTracker) {
    }

    public static record ObjectDetectionCamera(
            ObjectDetectionIO io,
            ObjectDetectionIOInputsAutoLogged inputs,
            VisionSource source,
            DuplicateTracker dupeTracker) {
    }

    private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();
    // There's only one Camera that will be detecting objects
    private ObjectDetectionCamera objectDetectionCamera;

    private ConcurrentLinkedQueue<VisionMeasurement> visionMeasurements = new ConcurrentLinkedQueue<>();

    private Optional<PhotonTrackedTarget> closetObject = Optional.empty();

    public Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    public VisionSubsystem() {
        // Initialize all cameras that we have pre-configured from VisionConstants.
        //
        // Using the APRIL_TAG_SOURCES constants, we read each camera established in the
        // Constants file and populate our camera_estimators available for April Tags.
        for (VisionConstants.VisionSource source : VisionConstants.APRIL_TAG_SOURCES) {
            AprilTagIO io;

            switch (AdvantageKitConstants.getMode()) {
                case REAL:
                    io = new AprilTagPhoton(source);
                    break;
                case SIM:
                    io = new AprilTagPhotonSim(source);
                    break;
                // case REPLAY:
                default:
                    io = new AprilTagIO() {
                    };
                    break;
            }
            aprilTagCameras
                    .add(new AprilTagCamera(
                            io,
                            new AprilTagIOInputsAutoLogged(),
                            source,
                            new DuplicateTracker()));
        }

        // Initialize the camera used exclusively for Object Detection (Notes)
        ObjectDetectionIO io;

        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ObjectDetectionPhoton(VisionConstants.OBJECT_DETECTION_SOURCE);
                break;
            case SIM:
                io = new ObjectDetectionPhotonSim(VisionConstants.OBJECT_DETECTION_SOURCE);
                break;
            default:
                io = new ObjectDetectionIO() {

                };
                break;
        }
        objectDetectionCamera = new ObjectDetectionCamera(io, new ObjectDetectionIOInputsAutoLogged(),
                VisionConstants.OBJECT_DETECTION_SOURCE, new DuplicateTracker());

    }

    // Enforce periodic method for VirtualSubsystem
    @Override
    public void periodic() {
        // Check for updates to Measurements from April Tags
        updateVisionMeasurements();

        // Check for updates to Measurements away from Notes
        updateClosestObject();
    }

    public void simulationPeriodic() {
        VisionConstants.VISION_SYSTEM_SIM.ifPresent((visionSystemSim) -> {
            visionSystemSim.update(robotPoseSupplier.get());
        });
    }

    /**
     * Using all available camera estimators, we poll each and try to build
     * our VisionMeasurements to help correct our position and odometry.
     */
    private void updateVisionMeasurements() {
        // For each camera we need to do the following:
        for (AprilTagCamera cam : aprilTagCameras) {
            String cameraLogRoot = "AprilTagCamera/" + cam.source.name() + "/";

            cam.io.updateInputs(cam.inputs);
            Logger.processInputs(cameraLogRoot, cam.inputs);

            // If we have a duplicate frame, don't bother updating anything
            if (cam.inputs.isDuplicateFrame) {
                continue;
            }

            Logger.recordOutput(cameraLogRoot + "Targets",
                    cam.inputs.frame.getTargets().stream()
                            .mapToInt(PhotonTrackedTarget::getFiducialId)
                            .distinct()
                            .toArray());

            // Add estimated position and deviation to be used by SwerveDrivePoseEstimator
            EstimatedPose estimatedPose = cam.inputs.estimatedPose;

            Logger.recordOutput(cameraLogRoot + "EstimatedPose", estimatedPose.pose);
            Logger.recordOutput(cameraLogRoot + "EstimatedPoseTimestamp", estimatedPose.timestamp);

            if (estimatedPose.isPresent) {
                // Find Vision Measurement and add it for our Queue if it exists
                AprilTagAlgorithms
                        .findVisionMeasurement(estimatedPose.asEstimatedRobotPose())
                        .ifPresent(visionMeasurements::add);
            }
        }
    }

    /**
     * Alternative strategy for Notes, we need to find _where_ the note is
     * and how we need to rotate the robot to be in-line with the note.
     */
    public void updateClosestObject() {
        String cameraLogRoot = "ObjectDetection/" + objectDetectionCamera.source.name() + "/";

        objectDetectionCamera.io.updateInputs(objectDetectionCamera.inputs);
        Logger.processInputs(cameraLogRoot, objectDetectionCamera.inputs);

        if (objectDetectionCamera.inputs.isDuplicateFrame) {
            return;
        }

        List<PhotonTrackedTarget> nonFiducialTargets = ObjectDetectionFiltering
                .getNonFiducialTargets(objectDetectionCamera.inputs.frame);

        // PhotonVision orders targets by "best", which in their `getBestTarget` method
        // just gets the first target, assuming there is a target. We just get that.
        // If there isn't a valid non-fiducial target, we display NaN.

        if (nonFiducialTargets.isEmpty()) {
            if (objectDetectionCamera.inputs.hasExceededTargetlessThreshold) {
                Logger.recordOutput(cameraLogRoot + "HasNotes", false);
                Logger.recordOutput(cameraLogRoot + "YawFromRobotCenterDeg", Double.NaN);
                closetObject = Optional.empty();
            }
        } else {
            PhotonTrackedTarget object = nonFiducialTargets.get(0);

            Logger.recordOutput(cameraLogRoot + "HasNotes", true);
            Logger.recordOutput(cameraLogRoot + "YawFromRobotCenterDeg", object.getYaw());

            closetObject = Optional.of(object);
        }
    }

    public Optional<PhotonTrackedTarget> getClosestObject() {
        return closetObject;
    }

    /**
     * Determines whether a "Note" is within the camera's sight
     */
    public Trigger cameraSeesObject() {
        return new Trigger(() -> getClosestObject().isPresent());
    }

    /**
     * Return the head of the Queue as a single measurement
     */
    public VisionMeasurement pollLatestVisionMeasurement() {
        return visionMeasurements.poll();
    }

    /**
     * Gets a list of the Fiducial IDs for April Tags we detect
     *
     * @return Pipe concatenated list of IDs
     */
    public String getVisibleTargets() {
        List<Integer> targetIds = new ArrayList<>();

        for (AprilTagCamera cam : aprilTagCameras) {
            PhotonPipelineResult frame = cam.inputs.frame;
            for (PhotonTrackedTarget target : frame.targets) {
                targetIds.add(target.getFiducialId());
            }
        }

        Set<Integer> uniqueIds = new HashSet<>(targetIds);
        List<Integer> organizedIds = new ArrayList<>(uniqueIds);

        Collections.sort(organizedIds);

        String visibleTargets = organizedIds.stream()
                .map(Object::toString)
                .reduce((s1, s2) -> s1 + " | " + s2)
                .orElse("");

        return visibleTargets;
    }

    /**
     * Eventually return whether or not we see "notes"
     */
    public boolean doWeSeeNotes() {
        return false;
    }
}
