// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.AdvantageKitConstants.Mode;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java
public class WheelRadiusCharacterization extends Command {
    private static final String logRoot = "Commands/WheelRadiusCharacterization/";

    private static final double characterizationSpeedRadPerSec = 0.1;

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;

        private Direction(int value) {
            this.value = value;
        }
    }

    private final DriveSubsystem drive;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositionsRad;

    private double currentEffectiveWheelRadiusMeters = 0.0;

    public WheelRadiusCharacterization(DriveSubsystem drive, Direction omegaDirection) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        addRequirements(drive);
    }

    /**
     * If on a real robot get the gyro reading as we don't want vision to interfere.
     * If in sim we need to use the pose rotation for testing as there is no gyro.
     *
     * @return Heading (as above) in radians
     */
    private double getHeading() {
        return AdvantageKitConstants.getMode() == Mode.REAL
                ? drive.getHeading().getRadians()
                : drive.getPose().getRotation().getRadians();
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = getHeading();
        accumGyroYawRads = 0.0;

        startWheelPositionsRad = drive.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drive.runVelocity(
                0.0,
                0.0,
                omegaLimiter.calculate(omegaDirection.value * characterizationSpeedRadPerSec));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(getHeading() - lastGyroYawRads);
        lastGyroYawRads = getHeading();
        double averageWheelPosition = 0.0;
        double[] wheelPositions = drive.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositionsRad[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadiusMeters = (accumGyroYawRads * DriveConstants.kRadius) / averageWheelPosition;
        Logger.recordOutput(logRoot + "DrivePosition", averageWheelPosition);
        Logger.recordOutput(logRoot + "AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
                logRoot + "CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadiusMeters));
        Logger.recordOutput(
                logRoot + "CurrentWheelDiameterInches",
                Units.metersToInches(2.0 * currentEffectiveWheelRadiusMeters));
        Logger.recordOutput(
                logRoot + "CurrentWheelRadiusMeters",
                currentEffectiveWheelRadiusMeters);
        Logger.recordOutput(
                logRoot + "CurrentWheelDiameterMeters",
                2.0 * currentEffectiveWheelRadiusMeters);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Interrupted early no data");
            return;
        }

        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            double radiusMeters = currentEffectiveWheelRadiusMeters;
            double diameterMeters = 2.0 * radiusMeters;
            System.out.println(
                    "Effective Wheel Radius (in): "
                            + Units.metersToInches(radiusMeters));
            System.out.println(
                    "Effective Wheel Diameter (in): "
                            + Units.metersToInches(diameterMeters));
            System.out.println("Effective Wheel Radius (meters): " + radiusMeters);
            System.out.println("Effective Wheel Diameter (meters): " + diameterMeters);
        }
        drive.runVelocity(0.0, 0.0, 0.0);
    }
}
