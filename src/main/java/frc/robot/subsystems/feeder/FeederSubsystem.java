// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.reusable_io.beambreak.BeambreakDigitalInput;
import frc.robot.reusable_io.beambreak.BeambreakIO;
import frc.robot.reusable_io.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.reusable_io.beambreak.BeambreakIOSim;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO io;
    private final BeambreakIO beambreak;

    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

    /** Creates a new ShooterSubsystem. */
    public FeederSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new FeederIOTalonFX();
                beambreak = new BeambreakDigitalInput(ShooterConstants.kBeambreakChannel);
                break;
            case SIM:
                io = new FeederIOSim() {
                };
                beambreak = new BeambreakIOSim(ShooterConstants.kBeambreakChannel);
                break;
            case REPLAY:
            default:
                io = new FeederIO() {
                };
                beambreak = new BeambreakIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        beambreak.updateInputs(beambreakInputs);

        Logger.processInputs("Feeder", inputs);
        Logger.processInputs("Feeder/BeamBreak", beambreakInputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        }
    }

    public void setVelocity(double velocityRotPerSecondFeeder) {
        io.setVelocity(velocityRotPerSecondFeeder);
    }

    public Command setVelocityCommand(double velocityRotPerSecond) {
        return new InstantCommand(() -> setVelocity(velocityRotPerSecond));
    }

    public Command setVelocityBeambreakCommand(double velocityRotPerSecond) {
        return new RunCommand(() -> io.setVelocity(velocityRotPerSecond), this)
                .unless(beambreakIsObstructed())
                .until(beambreakIsObstructed())
                .finallyDo(io::stop);
    }

    public Command stopCommand() {
        return new InstantCommand(io::stop, this);
    }

    // For testing and sim
    public Command setBeambreakObstructedCommand(boolean value) {
        return new InstantCommand(() -> {
            beambreak.overrideObstructed(value);
        });
    }

    // For testing and sim
    public Command toggleBeamBrakeObstructedCommand() {
        return new InstantCommand(() -> {
            beambreak.overrideObstructed(!beambreakInputs.isObstructed);
        });
    }

    public Trigger beambreakIsObstructed() {
        return new Trigger(() -> beambreakInputs.isObstructed);
    }

    public Command feedIntoAmpCommand() {
        return new InstantCommand(() -> {
            setVelocity(FeederConstants.kAmpVelocity);
        }, this);
    }
}
