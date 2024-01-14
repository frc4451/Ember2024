package frc.robot.subsystems.vision.apriltag;

import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO {
    @AutoLog
    public static class AprilTagIOInputs {
        public PhotonPipelineResult frame = new PhotonPipelineResult();
        @Nullable
        public Pose3d estimatedPose = null;
        public double estimatedPoseTimestamp = -1.0;
        // @Nullable
        // public PhotonTrackedTarget[] estimatedPoseTargetsUsed = null;
    }

    public default void updateInputs(AprilTagIOInputs inputs) {
    }
}
