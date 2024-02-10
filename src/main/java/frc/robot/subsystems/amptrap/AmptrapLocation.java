package frc.robot.subsystems.amptrap;

import edu.wpi.first.math.geometry.Rotation2d;

public enum AmptrapLocation {
    INITIAL(Rotation2d.fromDegrees(0.0));

    public Rotation2d angle;

    AmptrapLocation(Rotation2d angle) {
        this.angle = angle;
    }
}
