package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.VisionSource;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagIOInputsAutoLogged;
import frc.robot.subsystems.vision.apriltag.AprilTagAlgorithms;
import frc.robot.subsystems.vision.apriltag.AprilTagPhoton;
import frc.utils.VirtualSubsystem;

public class VisionSubsystem extends VirtualSubsystem {
    public static record VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {
    }

    public static record AprilTagCamera(AprilTagIO io, AprilTagIOInputsAutoLogged inputs, VisionSource source) {
    }

    private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

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
                // case REPLAY:
                default:
                    io = new AprilTagIO() {
                    };
                    break;
            }
            aprilTagCameras.add(new AprilTagCamera(io, new AprilTagIOInputsAutoLogged(), source));
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
        for (AprilTagCamera cam : aprilTagCameras) {
            cam.io.updateInputs(cam.inputs);
            Logger.processInputs("AprilTagCamera/" + cam.source.name(), cam.inputs);

            // Add estimated position and deviation to be used by SwerveDrivePoseEstimator
            cam.io.estimateRobotPose(cam.inputs.frame)
                    .flatMap(AprilTagAlgorithms::findVisionMeasurement)
                    .ifPresent(visionMeasurements::add);
        }
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
}
