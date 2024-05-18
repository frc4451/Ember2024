package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class StrafeAndAimToPose extends Command {
    private final PIDController thetaController = new PIDController(5, 0, 0.1);
    private final String logRoot;

    private final DriveSubsystem drive;
    private final Supplier<Pose3d> poseSupplier;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final boolean fieldRelative;

    private final double yawMeasurementOffset;
    private double yawErrorRad;
    private Pose3d targetPose = new Pose3d();
    private boolean hasSeenTag = false;

    public StrafeAndAimToPose(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Pose3d> poseSupplier,
            DriveSubsystem drive,
            boolean fieldRelative) {
        this(xSupplier, ySupplier, poseSupplier, drive, fieldRelative, Math.PI);
    }

    public StrafeAndAimToPose(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Pose3d> poseSupplier,
            DriveSubsystem drive,
            boolean fieldRelative,
            double yawMeasurementOffset) {
        addRequirements(drive);
        setName("StrafeAndAimToAprilTag");

        logRoot = "Commands/" + getName() + "/";

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.poseSupplier = poseSupplier;
        this.drive = drive;
        this.fieldRelative = fieldRelative;
        this.yawMeasurementOffset = yawMeasurementOffset;
        this.yawErrorRad = yawMeasurementOffset;

        thetaController.setTolerance(Units.degreesToRadians(0.1));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StrafeAndAimToPose(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Pose3d> poseSupplier,
            DriveSubsystem drive) {
        this(xSupplier, ySupplier, poseSupplier, drive, true);
    }

    @Override
    public void initialize() {
        drive.runVelocity(new ChassisSpeeds());
        targetPose = poseSupplier.get();

        Logger.recordOutput(logRoot + "IsRunning", true);
    }

    @Override
    public void execute() {
        targetPose = poseSupplier.get();
        Pose3d robotPose = new Pose3d(drive.getPose());

        yawErrorRad = targetPose.relativeTo(robotPose).getTranslation().toTranslation2d().getAngle().getRadians();

        double rotationSpeedRad = thetaController.calculate(yawMeasurementOffset, yawErrorRad);

        double offsetYawRad = yawErrorRad + yawMeasurementOffset;

        Logger.recordOutput(logRoot + "TargetPose", targetPose);
        Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
        Logger.recordOutput(logRoot + "TargetYawRad", yawErrorRad);
        Logger.recordOutput(logRoot + "TargetYawDeg", Units.radiansToDegrees(yawErrorRad));
        Logger.recordOutput(logRoot + "OffsetYawRad", offsetYawRad);
        Logger.recordOutput(logRoot + "OffsetYawDeg", Units.radiansToDegrees(offsetYawRad));
        Logger.recordOutput(logRoot + "RotationSpeed", rotationSpeedRad);

        TeleopDrive.drive(
                drive,
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                rotationSpeedRad / DriveConstants.kMaxAngularSpeed,
                false,
                fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(logRoot + "IsRunning", false);
    }
}
