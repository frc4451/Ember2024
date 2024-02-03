package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.GarageUtils;
import frc.utils.SwerveUtils;

public class TeleopDrive {
    // Slew rate filter variables for controlling lateral acceleration
    private static double currentRotation = 0.0;
    private static double currentTranslationDir = 0.0;
    private static double currentTranslationMag = 0.0;

    private static SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private static SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private static double prevTime = WPIUtilJNI.now() * 1e-6;

    public static Command asCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            boolean fieldRelative,
            boolean rateLimit) {
        return Commands.run(() -> drive(
                driveSubsystem,
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                omegaSupplier.getAsDouble(),
                fieldRelative,
                rateLimit),
                driveSubsystem);
    }

    public static void drive(
            DriveSubsystem driveSubsystem,
            double xSpeed,
            double ySpeed,
            double rot,
            boolean rateLimit,
            boolean fieldRelative) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                    // checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0.0);
                } else {
                    currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                }
            } else {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(
                        currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(0.0);
            }
            prevTime = currentTime;

            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        // Convert to field relative speeds & send command
        // (Copied from AdvantageKit example projects)
        boolean isFlipped = GarageUtils.getAlliance() == Alliance.Red;

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeedDelivered,
                        ySpeedDelivered,
                        rotDelivered,
                        isFlipped
                                ? driveSubsystem.getPose().getRotation().plus(new Rotation2d(Math.PI))
                                : driveSubsystem.getPose().getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        driveSubsystem.runVelocity(speeds);
    }
}
