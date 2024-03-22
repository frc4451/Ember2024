package frc.robot.bobot_state.TargetAngleTrackers;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;

public class NoteAngleTracker extends TargetAngleTracker {
    private boolean hasSeenNote = false;
    private Optional<Rotation2d> rotationTarget = Optional.empty();

    public NoteAngleTracker() {
        super();
    }

    public Optional<Rotation2d> getRotationTarget() {
        return this.rotationTarget;
    }

    public boolean getHasSeenNote() {
        return this.hasSeenNote;
    }

    public void update() {
        Optional<PhotonTrackedTarget> maybeTarget = BobotState.getClosestObject();

        maybeTarget.ifPresentOrElse((PhotonTrackedTarget target) -> {
            this.hasSeenNote = true;
            this.rotationTarget = Optional.of(
                    BobotState
                            .getRobotPose()
                            .getRotation()
                            .plus(Rotation2d.fromDegrees(-target.getYaw())));
        }, () -> {
            this.rotationTarget = Optional.empty();
        });
    }
}
