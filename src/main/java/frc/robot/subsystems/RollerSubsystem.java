package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RollerSubsystem extends SubsystemBase {
    private final TalonFX topRoller = new TalonFX(IntakeConstants.kTopRollerCanId);
    private final TalonFX bottomRoller = new TalonFX(IntakeConstants.kBottomRollerCanId);

    private final DigitalInput beamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);

    public RollerSubsystem() {
        this.topRoller.setInverted(true);
        for (TalonFX roller : List.of(this.topRoller, this.bottomRoller)) {
            TalonFXConfigurator configurator = roller.getConfigurator();
            configurator.apply(new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake))
                    .withCurrentLimits(IntakeConstants.rollerCurrentConfig));
        }
    }

    private void runRollersPercent(double topPercent, double bottomPercent) {
        this.topRoller.set(topPercent);
        this.bottomRoller.set(bottomPercent);
    }

    public boolean isBeamBreakActivated() {
        return !this.beamBreak.get();
    }

    public void runRollers(RollerMode rollerMode) {
        if (rollerMode.isLimited && this.isBeamBreakActivated()) {
            this.runRollersPercent(0.0, 0.0);
        } else {
            this.runRollersPercent(rollerMode.topPercent, rollerMode.bottomPercent);
        }
    }

    public Command runRollersCommand(RollerMode rollerMode) {
        return new InstantCommand(() -> this.runRollers(rollerMode), this);
    }
}
