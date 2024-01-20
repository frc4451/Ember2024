package frc.robot.subsystems.Misc;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This subsystem is for misc stuff allred wants us to make
 * currently occupied by vortex velocity control
 */
public class MiscSubsystem extends SubsystemBase {
    private final CANSparkFlex vortex;

    public MiscSubsystem() {
        vortex = new CANSparkFlex(2, MotorType.kBrushless);
    }

    private void setPercent(double velocityPercent) {
        vortex.set(velocityPercent);
    }

    public Command setVelocityCommand(double velocityPercent) {
        return new InstantCommand(() -> setPercent(velocityPercent));
    }

    public Command stopCommand() {
        return new InstantCommand(() -> setPercent(0.0));
    }
}
