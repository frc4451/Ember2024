package frc.robot.bobot_state.TargetAngleTrackers;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;

public class NoteAngleTracker extends TargetAngleTracker {
    private boolean hasSeenNote = false;
    private Rotation2d targetRotation = new Rotation2d();

    public NoteAngleTracker() {
        super();
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
        Optional<PhotonTrackedTarget> maybeTarget = BobotState.getClosestObject();

        maybeTarget.ifPresent((PhotonTrackedTarget target) -> {
            this.hasSeenNote = true;
            this.targetRotation = BobotState
                    .getRobotPose()
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(-target.getYaw()));
        });
    }
}
