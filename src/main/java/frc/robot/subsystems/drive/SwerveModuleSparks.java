// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.utils.SparkoidBurnManager;

public class SwerveModuleSparks implements SwerveModuleIO {
    private final CANSparkFlex m_drivingSparkFlex;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final SparkPIDController m_turningPIDController;

    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a Swerve Module and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, a SPARK Flex, and a Through Bore
     * Encoder.
     */
    public SwerveModuleSparks(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_chassisAngularOffset = chassisAngularOffset;

        m_drivingSparkFlex = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSparkFlex.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkFlex.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();

        if (SparkoidBurnManager.shouldBurn()) {
            // Factory reset, so we get the SPARK Flexes to a known state before configuring
            // them. This is useful in case a SPARK Flex is swapped out.
            m_drivingSparkFlex.restoreFactoryDefaults();
            m_turningSparkMax.restoreFactoryDefaults();
        }

        m_drivingSparkFlex.setCANTimeout(SparkoidBurnManager.kConfigCANTimeout);
        m_turningSparkMax.setCANTimeout(SparkoidBurnManager.kConfigCANTimeout);

        for (int i = 0; i < SparkoidBurnManager.kConfigAttempts; i++) {
            // Setup encoders and PID controllers for the driving and turning SPARK Flexes.
            m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
            m_turningPIDController.setFeedbackDevice(m_turningEncoder);

            // Apply position and velocity conversion factors for the driving encoder. The
            // native units for position and velocity are rotations and RPM, respectively,
            // but we want meters and meters per second to use with WPILib's swerve APIs.
            m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
            m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

            // Apply position and velocity conversion factors for the turning encoder. We
            // want these in radians and radians per second to use with WPILib's swerve
            // APIs.
            m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
            m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of
            // the steering motor in the FlexSwerve Module.
            m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            m_turningPIDController.setPositionPIDWrappingEnabled(true);
            m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
            m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

            // Set the PID gains for the driving motor. Note these are example gains, and
            // you
            // may need to tune them for your own robot!
            m_drivingPIDController.setP(ModuleConstants.kDrivingP);
            m_drivingPIDController.setI(ModuleConstants.kDrivingI);
            m_drivingPIDController.setD(ModuleConstants.kDrivingD);
            m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
            m_drivingPIDController.setOutputRange(
                    ModuleConstants.kDrivingMinOutput,
                    ModuleConstants.kDrivingMaxOutput);

            // Set the PID gains for the turning motor. Note these are example gains, and
            // you
            // may need to tune them for your own robot!
            m_turningPIDController.setP(ModuleConstants.kTurningP);
            m_turningPIDController.setI(ModuleConstants.kTurningI);
            m_turningPIDController.setD(ModuleConstants.kTurningD);
            m_turningPIDController.setFF(ModuleConstants.kTurningFF);
            m_turningPIDController.setOutputRange(
                    ModuleConstants.kTurningMinOutput,
                    ModuleConstants.kTurningMaxOutput);

            m_drivingSparkFlex.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
            m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
            m_drivingSparkFlex.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
            // m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit,
            // 20);
            // m_turningSparkMax.setSecondaryCurrentLimit(insertThing);

            m_drivingEncoder.setPosition(0);
        }

        if (SparkoidBurnManager.shouldBurn()) {
            // Save the SPARK Flex configurations. If a SPARK Flex browns out during
            // operation, it will maintain the above configurations.
            m_drivingSparkFlex.burnFlash();
            m_turningSparkMax.burnFlash();
        }

        // Set timeouts back to the default
        m_drivingSparkFlex.setCANTimeout(0);
        m_turningSparkMax.setCANTimeout(0);

        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = m_drivingEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = m_drivingEncoder.getVelocity();
        inputs.driveAppliedDutyCycle = m_drivingSparkFlex.getAppliedOutput();
        inputs.driveAppliedVoltage = m_drivingSparkFlex.getAppliedOutput() * m_drivingSparkFlex.getBusVoltage();
        inputs.driveCurrentAmps = m_drivingSparkFlex.getOutputCurrent();
        inputs.driveTemperatureCelsius = m_drivingSparkFlex.getMotorTemperature();

        inputs.turnAbsolutePositionRad = m_turningEncoder.getPosition();
        inputs.turnAngularOffsetPositionRad = m_turningEncoder.getPosition() - m_chassisAngularOffset;
        inputs.turnVelocityRadPerSec = m_turningEncoder.getVelocity();
        inputs.turnAppliedDutyCycle = m_turningSparkMax.getAppliedOutput();
        inputs.turnAppliedVoltage = m_turningSparkMax.getAppliedOutput() * m_turningSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = m_turningSparkMax.getOutputCurrent();
        inputs.turnTemperatureCelsius = m_turningSparkMax.getMotorTemperature();

        Rotation2d angle = new Rotation2d(inputs.turnAngularOffsetPositionRad);
        inputs.state = new SwerveModuleState(inputs.driveVelocityMetersPerSec, angle);
        inputs.position = new SwerveModulePosition(inputs.drivePositionMeters, angle);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle.plus(new Rotation2d(m_chassisAngularOffset)));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
                correctedDesiredState,
                new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS Flex towards their respective setpoints.
        m_drivingPIDController.setReference(
                optimizedDesiredState.speedMetersPerSecond,
                CANSparkFlex.ControlType.kVelocity);

        m_turningPIDController.setReference(
                optimizedDesiredState.angle.getRadians(),
                CANSparkFlex.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    @Override
    public void runDriveCharacterization(double voltage) {
        m_drivingSparkFlex.setVoltage(voltage);

        m_turningPIDController.setReference(
                m_chassisAngularOffset,
                CANSparkFlex.ControlType.kPosition);
    }
}
