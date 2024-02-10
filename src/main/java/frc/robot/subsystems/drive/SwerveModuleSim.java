// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleSim implements SwerveModuleIO {
    /**
     * Main timer to simulate the passage of time.
     */
    private final Timer timer = new Timer();

    /**
     * Time delta since last update
     */
    private double dt;

    /**
     * Last time queried.
     */
    private double lastTime;

    /**
     * Fake motor position.
     */
    private double fakePos;

    /**
     * The fake speed of the previous state, used to calculate
     * {@link SwerveModuleSimulation#fakePos}.
     */
    private double fakeSpeed;

    /**
     * Current simulated swerve module state.
     */
    private SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0));

    private final double chassisAngularOffset;

    public SwerveModuleSim(double chassisAngularOffset) {
        this.chassisAngularOffset = chassisAngularOffset;

        this.timer.start();
        this.lastTime = timer.get();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = fakePos;
        inputs.driveVelocityMetersPerSec = fakeSpeed;

        inputs.turnAbsolutePositionRad = state.angle.getRadians();
        inputs.turnAngularOffsetPositionRad = state.angle.getRadians() - chassisAngularOffset;

        Rotation2d angle = new Rotation2d(inputs.turnAngularOffsetPositionRad);
        inputs.state = new SwerveModuleState(fakeSpeed, angle);
        inputs.position = new SwerveModulePosition(fakePos, angle);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState angularOffsetState = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle.plus(new Rotation2d(chassisAngularOffset)));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(angularOffsetState, state.angle);

        // Find timer stuff for updating the fake pos & speed
        double currentTime = timer.get();

        dt = currentTime - lastTime;
        lastTime = currentTime;

        // Set the state and fake pos & speed
        fakeSpeed = optimizedState.speedMetersPerSecond;
        fakePos += fakeSpeed * dt;

        state = optimizedState;
    }
}
