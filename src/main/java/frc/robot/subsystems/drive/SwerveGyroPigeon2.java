package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class SwerveGyroPigeon2 implements SwerveGyroIO {
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveConstants.kGyroCanId);

    public void updateInputs(SwerveGyroIOInputs inputs) {
        inputs.isConnected = this.pigeon.getLastError().equals(ErrorCode.OK);
        inputs.yawPositionRad = Units.degreesToRadians(-this.pigeon.getAngle());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-this.pigeon.getRate());
    }

    public void zero() {
        this.pigeon.reset();
    }
}
