package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RollerSubsystem extends SubsystemBase {
    private final WPI_TalonFX topRoller = new WPI_TalonFX(IntakeConstants.kTopRollerCanId);
    private final WPI_TalonFX bottomRoller = new WPI_TalonFX(IntakeConstants.kBottomRollerCanId);

    private final DigitalInput beamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);

    public RollerSubsystem() {
        for (WPI_TalonFX roller : List.of(this.topRoller, this.bottomRoller)) {
            roller.configFactoryDefault();
            roller.setNeutralMode(NeutralMode.Brake);
            roller.configSupplyCurrentLimit(IntakeConstants.rollerCurrentConfig);
        }
    }

    private void runRollersPercent(double topPercent, double bottomPercent) {
        this.topRoller.set(ControlMode.PercentOutput, topPercent);
        this.bottomRoller.set(ControlMode.PercentOutput, bottomPercent);
    }

    private boolean isBeamBreakActivated() {
        return this.beamBreak.get();
    }

    public void runRollers(RollerMode rollerMode) {
        if (rollerMode.isLimited && this.isBeamBreakActivated()) {
            this.runRollersPercent(0, 0);
        } else {
            this.runRollersPercent(rollerMode.topPercent, rollerMode.bottomPercent);
        }
    }

    public Command runRollersCommand(RollerMode rollerMode) {
        return new InstantCommand(() -> this.runRollers(rollerMode), this);
    }
}
