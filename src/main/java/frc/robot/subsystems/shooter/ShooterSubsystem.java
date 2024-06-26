// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.bobot_state.interpolation.ShootingInterpolator;

public class ShooterSubsystem extends SubsystemBase {
    private double targetVelocityRotPerSecondLeft = 0;
    private double targetVelocityRotPerSecondRight = 0;

    private final ShooterIO io;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new ShooterIOTalonFX();
                break;
            case SIM:
                io = new ShooterIOSim() {
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

    public void setVelocity(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        io.setVelocity(velocityRotPerSecondLeft, velocityRotPerSecondRight);
        this.targetVelocityRotPerSecondLeft = velocityRotPerSecondLeft;
        this.targetVelocityRotPerSecondRight = velocityRotPerSecondRight;

    }

    public Command setVelocityCommand(double velocityRotPerSecondLeft, double velocityRotPerSecondRight) {
        return new InstantCommand(() -> setVelocity(velocityRotPerSecondLeft, velocityRotPerSecondRight), this);
    }

    public Command stopCommand() {
        return new InstantCommand(io::stop, this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(io::stop, this);
    }

    public Command rampUpSpeedToSpeakerCommand() {
        return new RunCommand(() -> {
            ShootingInterpolator.InterpolatedCalculation shootingCalculation = BobotState.getSpeakerCalculation();
            setVelocity(shootingCalculation.leftSpeedRotPerSec(), shootingCalculation.rightSpeedRotPerSec());
        }, this).andThen(stopCommand());
    }

    public Command rampUpSpeedToFloorCommand() {
        return new RunCommand(() -> {
            ShootingInterpolator.InterpolatedCalculation shootingCalculation = BobotState.getFloorCalculation();
            setVelocity(shootingCalculation.leftSpeedRotPerSec(), shootingCalculation.rightSpeedRotPerSec());
        }, this).andThen(stopCommand());
    }

    public Command rampUpSpeedToPoopCommand() {
        return new RunCommand(() -> {
            setVelocity(Constants.ShooterConstants.kLeftShooterPoopSpeed,
                    Constants.ShooterConstants.kRightShooterPoopSpeed);
        }, this).andThen(stopCommand());
    }

    public Command shootIntoAmpCommand() {
        return new InstantCommand(() -> setVelocity(ShooterConstants.kAmpSpeed, ShooterConstants.kAmpSpeed), this);
    }

    public Trigger shooterNearTarget() {
        return new Trigger(
                () -> MathUtil.isNear(inputs.velocityRotPerSecondLeft, this.targetVelocityRotPerSecondLeft, 1.0) &&
                        MathUtil.isNear(inputs.velocityRotPerSecondRight, this.targetVelocityRotPerSecondRight, 1.0));
    }
}
