// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.reusable_io.beambreak.BeambreakSim;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final BeambreakIO beambreak;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ShooterIOTalonFX();
                beambreak = new BeambreakDigitalInput(ShooterConstants.kBeamBreakCanID);
                break;
            case SIM:
                io = new ShooterIOSim() {
                };
                beambreak = new BeambreakSim(ShooterConstants.kBeamBreakCanID);
                break;
            case REPLAY:
            default:
                io = new ShooterIO() {
                };
                beambreak = new BeambreakIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        this.beambreak.updateInputs(this.beambreakInputs);

        Logger.processInputs("Shooter", this.inputs);
        Logger.processInputs("Shooter/BeamBreak", this.beambreakInputs);

        if (DriverStation.isDisabled()) {
            io.stopShooter();
            io.stopFeeder();
        }
    }

    public Command setVelocityFeederCommand(double velocityRotPerSecondFeeder) {
        return new InstantCommand(
                () -> this.io.setVelocityFeeder(velocityRotPerSecondFeeder), this);
    }

    public Command setVelocityShooterCommand(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        return new InstantCommand(
                () -> this.io.setVelocityShooter(velocityRotPerSecondLeft, velocityRotPerSecondRight), this);
    }

    public Command stopCommand() {
        return new InstantCommand(this.io::stop, this);
    }

    public Command stopFeederCommand() {
        return new InstantCommand(this.io::stopFeeder, this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(this.io::stopShooter, this);
    }

    // For testing and sim
    public Command setBeamBreakActivatedCommand(boolean value) {
        return new InstantCommand(() -> {
            this.beambreak.overrideActivated(value);
        });
    }

    // For testing and sim
    public Command toggleBeamBrakeActivatedCommand() {
        return new InstantCommand(() -> {
            this.beambreak.overrideActivated(!this.beambreakInputs.isActivated);
        });
    }

    public Trigger beambreakIsActivated() {
        return new Trigger(() -> this.beambreakInputs.isActivated);
    }
}
