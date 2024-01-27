package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PathfindToTarget extends Command {
    private final PIDController xController = new PIDController(1.5, 0, 0);
    private final PIDController thetaController = new PIDController(5, 0, 0);

    private final Supplier<Optional<PhotonTrackedTarget>> targetSupplier;

    private final DriveSubsystem drive;

    private final Timer driveExtraTimer = new Timer();

    private boolean shouldFinish = false;

    private int step = 1;

    public PathfindToTarget(
            Supplier<Optional<PhotonTrackedTarget>> targetSupplier,
            DriveSubsystem drive) {
        this.targetSupplier = targetSupplier;
        this.drive = drive;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, false, true);
    }

    @Override
    public void execute() {
        switch (step) {
            case 1 -> stepOne();
            case 2 -> stepTwo();
            default -> shouldFinish = true;
        }
    }

    private void stepOne() {
        Optional<PhotonTrackedTarget> maybeTarget = targetSupplier.get();
        if (maybeTarget.isEmpty()) {
            step++;
            return;
        }

        PhotonTrackedTarget target = maybeTarget.get();

        Transform3d robotToCamera = VisionConstants.OBJECT_DETECTION_SOURCE.robotToCamera();

        // The distance in a straight line from the camera to the target
        double distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(),
                Units.inchesToMeters(2),
                -robotToCamera.getRotation().getY(),
                Units.degreesToRadians(target.getPitch()));

        Translation2d cameraToTarget = PhotonUtils.estimateCameraToTargetTranslation(
                distanceToTargetMeters,
                Rotation2d.fromDegrees(target.getYaw()));

        // The transform to the target from the robot
        Transform2d robotToTarget = new Transform2d(
                robotToCamera.getTranslation().toTranslation2d().plus(cameraToTarget),
                robotToCamera.getRotation().toRotation2d().plus(cameraToTarget.getAngle()));

        double robotToTargetRadians = robotToTarget.getRotation().getRadians();

        double x = xController.calculate(0, robotToTarget.getTranslation().getNorm());
        double theta = thetaController.calculate(
                Math.PI,
                robotToTargetRadians);

        // Theta should be negative for real, positive for sim, dunno why
        if (AdvantageKitConstants.getMode() == AdvantageKitConstants.Mode.SIM) {
            theta = -theta;
        }

        drive.runVelocity(new ChassisSpeeds(-x, 0, -theta));
    }

    private void stepTwo() {
        driveExtraTimer.start();

        if (!driveExtraTimer.hasElapsed(0.25)) {
            drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0));
        } else {
            step++;
        }
    }

    @Override
    public boolean isFinished() {
        return shouldFinish;
    }
}
