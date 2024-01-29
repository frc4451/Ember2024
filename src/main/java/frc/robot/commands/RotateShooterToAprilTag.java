package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Command for centering the robot relative to an April Tag that the user
 * specifies. The ID must be a valid Fiducial ID.
 */
public class RotateShooterToAprilTag extends Command {
    private final PIDController thetaController = new PIDController(1, 0, 0);

    private final DriveSubsystem drive;
    private final int targetFiducialId;

    public RotateShooterToAprilTag(
            DriveSubsystem drive,
            int targetFiducialId) {
        this.drive = drive;
        this.targetFiducialId = targetFiducialId;
    }

    @Override
    public void execute() {
        AprilTag targetTag = VisionConstants.FIELD_LAYOUT.getTags().get(this.targetFiducialId);
        double targetYaw = PhotonUtils.getYawToPose(
                drive.getPose(),
                targetTag.pose.toPose2d())
                // Shoot out the back?
                .rotateBy(Rotation2d.fromRadians(Math.PI))
                .getDegrees();

        double rotationSpeed = -thetaController.calculate(targetYaw, 0);

        // @NOTE Use this for debugging, remove at competition
        Logger.recordOutput("RotateToAprilTag/TargetYaw", targetYaw);
        Logger.recordOutput("RotateToAprilTag/rotationSpeed", rotationSpeed);

        drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
    }
}
