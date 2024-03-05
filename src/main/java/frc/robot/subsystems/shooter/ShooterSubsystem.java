// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.bobot_state.ShootingInterpolator;

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
                io = new ShooterIOSim();
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
        this.io.updateInputs(inputs);

        Logger.processInputs("Shooter", this.inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        }
    }

    public void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        this.io.setVelocity(velocityRotPerSecondLeft, velocityRotPerSecondRight);
    }

    public Command setVelocityCommand(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        return new InstantCommand(
                () -> setVelocity(velocityRotPerSecondLeft, velocityRotPerSecondRight), this);
    }

    public Command stopCommand() {
        return new InstantCommand(this.io::stop, this);
    }

    public Command shootAtSpeakerCommand() {
        return new RunCommand(() -> {
            ShootingInterpolator.InterpolatedCalculation shootingCalculation = BobotState.getShootingCalculation();
            setVelocity(shootingCalculation.leftSpeedRotPerSec(), shootingCalculation.rightSpeedRotPerSec());
        }, this).andThen(stopCommand());
    }
}
