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
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.reusable_io.beambreak.BeambreakIOSim;

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
                beambreak = new BeambreakDigitalInput(Constants.ShooterConstants.kBeamBreakCanID);
                break;
            case SIM:
                io = new ShooterIOSim() {
                };
                beambreak = new BeambreakIOSim(Constants.ShooterConstants.kBeamBreakCanID);
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
            io.stop();
        }
    }

    public void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight,
            double velocityRotPerSecondFeeder) {
        io.setVelocity(velocityRotPerSecondLeft, velocityRotPerSecondRight, velocityRotPerSecondFeeder);
    }

    public Command setVelocityCommand(double velocityRotPerSecondLeft, double velocityRotPerSecondRight,
            double velocityRotPerSecondFeeder) {
        return new InstantCommand(
                () -> setVelocity(velocityRotPerSecondLeft, velocityRotPerSecondRight, velocityRotPerSecondFeeder),
                this);
    }

    public void stop() {
        io.stop();
    }

    public Command stopCommand() {
        return new InstantCommand(() -> stop(), this);
    }

    public Command overrideBeamBreakActivatedCommand(boolean value) {
        return new InstantCommand(() -> {
            this.beambreak.overrideActivated(value);
        });
    }

    public Trigger beambreakIsActivated() {
        return new Trigger(() -> this.beambreakInputs.isActivated);
    }
}
