package frc.robot.subsystems.quest;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface QuestIO {
    @AutoLog
    public static class QuestIOInputs {
        /**
         * Pose with compensation constant and relative pose offset applied
         * (probably what you want)
         */
        public Pose2d relativePose = new Pose2d();
        public Pose2d compensatedPose = new Pose2d();
        public double yawRad = 0;

        public double timestamp = 0;
        public double batteryLevel = 0;

        public float[] rawPosition = { 0.0f, 0.0f, 0.0f };
        public float[] rawQuaternion = { 0.0f, 0.0f, 0.0f, 0.0f };
    }

    public default void updateInputs(QuestIOInputs inputs) {
    }

    /** Sets supplied pose as origin of all calculations */
    public default void resetPose(Pose2d pose) {
    }
}
