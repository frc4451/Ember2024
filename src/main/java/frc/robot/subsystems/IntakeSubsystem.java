package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX topRoller = new WPI_TalonFX(IntakeConstants.kTopRollerCanId);
    private final WPI_TalonFX bottomRoller = new WPI_TalonFX(IntakeConstants.kBottomRollerCanId);

    private final WPI_TalonFX pivot = new WPI_TalonFX(IntakeConstants.kPivotCanId);

    public void runRollers(double percentOutput) {
        this.topRoller.set(ControlMode.PercentOutput, percentOutput);
        this.bottomRoller.set(ControlMode.PercentOutput, percentOutput);
    }
}
