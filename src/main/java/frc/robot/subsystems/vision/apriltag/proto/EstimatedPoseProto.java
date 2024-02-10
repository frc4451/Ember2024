package frc.robot.subsystems.vision.apriltag.proto;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.proto.Apriltag.ProtobufEstimatedPose;
import frc.robot.subsystems.vision.apriltag.EstimatedPose;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class EstimatedPoseProto implements Protobuf<EstimatedPose, ProtobufEstimatedPose> {
    @Override
    public Class<EstimatedPose> getTypeClass() {
        return EstimatedPose.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufEstimatedPose.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] { EstimatedPose.proto };
    }

    @Override
    public ProtobufEstimatedPose createMessage() {
        return ProtobufEstimatedPose.newInstance();
    }

    @Override
    public EstimatedPose unpack(ProtobufEstimatedPose msg) {
        return new EstimatedPose(
                msg.getIsPresent(),
                Pose3d.proto.unpack(msg.getPose()),
                msg.getTimestamp(),
                PhotonTrackedTarget.proto.unpack(msg.getTargets()));
    }

    @Override
    public void pack(ProtobufEstimatedPose msg, EstimatedPose value) {
        msg.setIsPresent(value.isPresent);
        Pose3d.proto.pack(msg.getMutablePose(), value.pose);
        msg.setTimestamp(value.timestamp);
        PhotonTrackedTarget.proto.pack(msg.getMutableTargets(), value.targets);
    }
}
