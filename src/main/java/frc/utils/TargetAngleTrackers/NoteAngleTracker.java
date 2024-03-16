package frc.utils.TargetAngleTrackers;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;

public class NoteAngleTracker extends TargetAngleTracker {
    private boolean hasSeenNote = false;
    private Rotation2d targetRotation = new Rotation2d();
    private Supplier<Optional<PhotonTrackedTarget>> objectDetectionSupplier;

    public NoteAngleTracker(Supplier<Optional<PhotonTrackedTarget>> objectDetectionSupplier) {
        super();
        this.objectDetectionSupplier = objectDetectionSupplier;
    }

    public Rotation2d getRotationDifference() {
        return getTargetRotation();
    }

    public boolean getHasSeenNote() {
        return this.hasSeenNote;
    }

    public Rotation2d getTargetRotation() {
        return this.targetRotation;
    }

    public void update() {
        Optional<PhotonTrackedTarget> maybeTarget = objectDetectionSupplier.get();

        maybeTarget.ifPresent((PhotonTrackedTarget target) -> {
            this.hasSeenNote = true;
            this.targetRotation = BobotState
                    .getRobotPose()
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(-target.getYaw()));

        });

        this.log();
    }

    public void log() {
        String logRoot = "TargetAngle/Note/";
        Logger.recordOutput(logRoot + "TargetRotation", this.targetRotation);
        Logger.recordOutput(logRoot + "TargetAngleRad", this.getRotationDifference().getRadians());
        Logger.recordOutput(logRoot + "TargetAngleDegrees", this.getRotationDifference().getDegrees());
        Logger.recordOutput(logRoot + "HasSeenNote", this.hasSeenNote);
    }
}
