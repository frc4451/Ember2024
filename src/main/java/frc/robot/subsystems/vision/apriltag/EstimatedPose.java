package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.apriltag.proto.EstimatedPoseProto;

public class EstimatedPose implements ProtobufSerializable {
    public final boolean isPresent;

    public final Pose3d pose;

    public final double timestamp;

    public final List<PhotonTrackedTarget> targets;

    /** Default constructor for when no values are set */
    public EstimatedPose() {
        isPresent = false;
        pose = new Pose3d();
        timestamp = -1.0;
        targets = new ArrayList<>();
    }

    public EstimatedPose(Optional<EstimatedRobotPose> estimatedRobotPose) {
        if (estimatedRobotPose.isPresent()) {
            EstimatedRobotPose estimate = estimatedRobotPose.get();
            isPresent = true;
            pose = estimate.estimatedPose;
            timestamp = estimate.timestampSeconds;
            targets = estimate.targetsUsed;
        } else {
            isPresent = false;
            pose = new Pose3d();
            timestamp = -1.0;
            targets = new ArrayList<>();
        }
    }

    public EstimatedPose(boolean isPresent, Pose3d pose, double timestamp, List<PhotonTrackedTarget> targets) {
        this.isPresent = isPresent;
        this.pose = pose;
        this.timestamp = timestamp;
        this.targets = targets;
    }

    public static final EstimatedPoseProto proto = new EstimatedPoseProto();
}
