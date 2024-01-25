// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ShooterIOTalonFX();
                break;
            case SIM:
                io = new ShooterIO() {
                };
                break;
            case REPLAY:
            default:
                io = new ShooterIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        }
    }

    public void setVelocity(double velocityRotPerSecond) {
        io.setVelocity(velocityRotPerSecond);
    }

    public double[] getVelocity() {
        return inputs.velocityRotPerSecond;
    }

    public Command setVelocityCommand(double velocityRotPerSecond) {
        return new InstantCommand(() -> setVelocity(velocityRotPerSecond), this);
    }

    public void stop() {
        io.setFree();
    }

    public Command stopCommand() {
        return new InstantCommand(() -> stop(), this);
    }
}
