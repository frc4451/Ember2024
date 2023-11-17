// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOInputs;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
    // Swerve Modules
    private final SwerveModuleIO[] m_modules = new SwerveModuleIO[4]; // FL, FR, RL, RR
    private final SwerveModuleIOInputsAutoLogged[] m_moduleInputs = new SwerveModuleIOInputsAutoLogged[] {
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
    };

    // Gyro
    private final SwerveGyroIO m_gyro;
    private final SwerveGyroIOInputsAutoLogged m_gyroInputs = new SwerveGyroIOInputsAutoLogged();

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry for tracking robot pose
    // We use these variables to keep track of the
    // previous values so we can get deltas for odometry
    private double[] m_lastModulePositionsMeters = new double[4];
    private double m_lastYawPositionRad = 0.0;
    private Pose2d m_odometryPose = new Pose2d();

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                m_modules[0] = new SwerveModuleSparkMax(
                        DriveConstants.kFrontLeftDrivingCanId,
                        DriveConstants.kFrontLeftTurningCanId,
                        DriveConstants.kFrontLeftChassisAngularOffset);

                m_modules[1] = new SwerveModuleSparkMax(
                        DriveConstants.kFrontRightDrivingCanId,
                        DriveConstants.kFrontRightTurningCanId,
                        DriveConstants.kFrontRightChassisAngularOffset);

                m_modules[2] = new SwerveModuleSparkMax(
                        DriveConstants.kRearLeftDrivingCanId,
                        DriveConstants.kRearLeftTurningCanId,
                        DriveConstants.kBackLeftChassisAngularOffset);

                m_modules[3] = new SwerveModuleSparkMax(
                        DriveConstants.kRearRightDrivingCanId,
                        DriveConstants.kRearRightTurningCanId,
                        DriveConstants.kBackRightChassisAngularOffset);

                m_gyro = new SwerveGyroPigeon2();

                break;

            case SIM:
                m_modules[0] = new SwerveModuleSim(DriveConstants.kFrontLeftChassisAngularOffset);
                m_modules[1] = new SwerveModuleSim(DriveConstants.kFrontRightChassisAngularOffset);
                m_modules[2] = new SwerveModuleSim(DriveConstants.kBackLeftChassisAngularOffset);
                m_modules[3] = new SwerveModuleSim(DriveConstants.kBackRightChassisAngularOffset);
                m_gyro = new SwerveGyroIO() {
                };
                break;

            case REPLAY:
            default:
                for (int i = 0; i < m_modules.length; i++) {
                    m_modules[i] = new SwerveModuleIO() {
                    };
                }
                m_gyro = new SwerveGyroIO() {
                };
                break;
        }
    }

    private SwerveModuleState getModuleState(SwerveModuleIOInputs inputs) {
        return new SwerveModuleState(
                inputs.driveVelocityMetersPerSec,
                new Rotation2d(inputs.turnAngularOffsetPositionRad));
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(m_moduleInputs)
                .map(inputs -> getModuleState(inputs))
                .toArray(SwerveModuleState[]::new);
    }

    private SwerveModulePosition getModulePosition(SwerveModuleIOInputs inputs) {
        return new SwerveModulePosition(
                inputs.drivePositionMeters,
                new Rotation2d(inputs.turnAngularOffsetPositionRad));
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(m_moduleInputs)
                .map(inputs -> getModulePosition(inputs))
                .toArray(SwerveModulePosition[]::new);
    }

    public SwerveModulePosition[] getModuleWheelDeltas() {
        SwerveModulePosition[] output = new SwerveModulePosition[m_moduleInputs.length];
        for (int i = 0; i < m_moduleInputs.length; i++) {
            output[i] = new SwerveModulePosition(
                    m_moduleInputs[i].drivePositionMeters - m_lastModulePositionsMeters[i],
                    new Rotation2d(m_moduleInputs[i].turnAngularOffsetPositionRad));
        }
        return output;
    }

    @Override
    public void periodic() {
        m_gyro.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", m_gyroInputs);

        // Update the modules' inputs and other data
        m_lastModulePositionsMeters = Arrays.stream(m_moduleInputs)
                .mapToDouble((inputs) -> inputs.drivePositionMeters)
                .toArray();

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].updateInputs(m_moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i), m_moduleInputs[i]);
        }

        Logger.getInstance().recordOutput("SwerveStates/Measured", getModuleStates());

        // Update the odometry pose
        SwerveModulePosition[] moduleDeltas = getModuleWheelDeltas();
        // The twist represents the motion of the robot since the last
        // loop cycle in x, y, and theta based on only the modules,
        // without the gyro. The gyro is always disconnected in simulation.
        // If the gyro is connected we'll use that to calculate our theta instead.
        Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
        if (m_gyroInputs.isConnected) {
            twist.dtheta = m_gyroInputs.yawPositionRad - m_lastYawPositionRad;
            m_lastYawPositionRad = m_gyroInputs.yawPositionRad;
        }
        // Apply the twist (change since last loop cycle) to the current pose
        m_odometryPose = m_odometryPose.exp(twist);
        Logger.getInstance().recordOutput("Odometry/Robot", m_odometryPose);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometryPose;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void setPose(Pose2d pose) {
        m_odometryPose = pose;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                getRotation())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        for (int i = 0; i < swerveModuleStates.length; i++) {
            m_modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setCross() {
        m_modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        for (int i = 0; i < desiredStates.length; i++) {
            m_modules[i].setDesiredState(desiredStates[i]);
        }
    }

    /** Zeroes the heading of the robot. */
    public void zeroRotation() {
        m_gyro.zero();
        if (!m_gyroInputs.isConnected) {
            m_gyroInputs.yawPositionRad = 0.0;
        }
    }

    /**
     * Returns the rotation of the robot.
     */
    public Rotation2d getRotation() {
        return m_odometryPose.getRotation();
    }

    /**
     * Returns the turn rate of the robot in radians per second.
     */
    public double getTurnRate() {
        return m_gyroInputs.yawVelocityRadPerSec;
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean resetOdometry, boolean stopAfter) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (resetOdometry) {
                        this.setPose(trajectory.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        this::getPose, // Pose supplier
                        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(AutoConstants.kPXController, 0, 0),
                        new PIDController(AutoConstants.kPYController, 0, 0),
                        new PIDController(AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these
                                                                                  // values for your robot. Leaving them
                                                                                  // 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        false, // Should the path be automatically mirrored depending on alliance color.
                               // Optional, defaults to true
                        this // Requires this drive subsystem
                ),
                new InstantCommand(() -> {
                    if (stopAfter) {
                        this.drive(0, 0, 0, false, false);
                    }
                }));
    }
}
