// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int pdp = 0;

    /**
     * Command Scheduler loopback
     */
    public static double loopback = 0.02;

    public static final class AdvantageKitConstants {
        public static enum Mode {
            REAL,
            REPLAY,
            SIM
        }

        private static Mode kfakeMode = Mode.SIM;

        public static Mode getMode() {
            return RobotBase.isReal() ? Mode.REAL : kfakeMode;
        }
    }

    public static final class PathPlannerConstants {
        public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

        public static final double kMaxAngularAcceleration = 4 * Math.PI; // This was made up
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.00; // This was made up

        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                DriveConstants.kMaxSpeedMetersPerSecond,
                PathPlannerConstants.kMaxAccelerationMetersPerSecondSquared,
                DriveConstants.kMaxAngularSpeed,
                5 * Math.PI);

        public static final PathConstraints TEST_PATH_CONSTRAINTS = new PathConstraints(
                1.0,
                PathPlannerConstants.kMaxAccelerationMetersPerSecondSquared,
                DriveConstants.kMaxAngularSpeed,
                5 * Math.PI);
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(24.5);
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(24.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 15;
        public static final int kRearLeftDrivingCanId = 17;
        public static final int kFrontRightDrivingCanId = 11;
        public static final int kRearRightDrivingCanId = 13;

        public static final int kFrontLeftTurningCanId = 14;
        public static final int kRearLeftTurningCanId = 16;
        public static final int kFrontRightTurningCanId = 10;
        public static final int kRearRightTurningCanId = 12;

        public static final int kGyroCanId = 1;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kProgrammerControllerPort = 2;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class PivotConstants {
        public static final double kPivotReduction = 80.0;

        public static final int kPivotLeaderCanId = 6;
        public static final int kPivotFollowerCanId = 7;

        // TrapezoidProfile Constraints
        public static final double kMaxVelocityRadiansPerSecond = Units.degreesToRadians(20.0);
        public static final double kMaxAccelerationRadiansPerSecondSquared = Units.degreesToRadians(20.0);
    }

    public static final class IntakeConstants {
        public static final int kBeambreakChannel = 0;

        public static final int kIntakeCanId = 1;
        public static final double kIntakeVelocity = 20.0;
    }

    public static final class AmpTrapConstants {
        public static final CurrentLimitsConfigs rollerCurrentConfig = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30.0)
                .withSupplyCurrentThreshold(35.0)
                .withSupplyTimeThreshold(0.5);

        public static final int kCanId = 9;

        public static final double kShootSpeed = 75.0;

        public static final int kBeambreakChannel = 2;
    }

    public static final class ShooterConstants {
        public static final int kLeftShooterCanID = 3;
        public static final int kRightShooterCanID = 4;
        public static final int kFeederCanID = 2;
        public static final int kBeambreakChannel = 1;

        public static final double kFeederIntakeVelocity = 20.0;
        public static final double kFeederShootVelocity = 50.0;
    }

    public static final class ClimberConstants {
        public static final double kMaxVelocityRotPerSecond = 10.0;
        public static final double kMaxHeightInches = 18.0;
        public static final double kMinHeightInches = 0.0;

        public static final double kClimberSpoolDiameter = 1.0;
        public static final double kClimberReduction = 25.0;

        public static final int kClimberCanId = 5;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        // public static final double kD = 1.0;
    }

    public static final class ElevatorConstants {
        public static final double kMinHeightInches = 0.0;
        public static final double kMaxHeightInches = 23.5;

        public static final double kAmpScoreHeightInches = 15.0;
        public static final double kTrapScoreHeightInches = kMaxHeightInches;

        public static final double kPivotClearanceHeightInches = 15.0;

        public static final double kElevatorSpoolDiameter = 0.7;
        public static final double kElevatorReduction = 5.0;

        public static final int kElevatorCanID = 8;

        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class PhoenixConstants {
        public static final int kDefaultStatusSignalFrequencyHz = 50;
        public static final int kStatusSignalFrequencyHz = 25;
    }
}
