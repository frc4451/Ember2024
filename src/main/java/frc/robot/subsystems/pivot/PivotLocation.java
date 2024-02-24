package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotLocation {
    INITIAL(Rotation2d.fromDegrees(25.0)),
    k26(Rotation2d.fromDegrees(26.0)),
    k36(Rotation2d.fromDegrees(36.0)),
    k55(Rotation2d.fromDegrees(55.0)),
    k85(Rotation2d.fromDegrees(85.0));

    public Rotation2d angle;

    PivotLocation(Rotation2d angle) {
        this.angle = angle;
    }
}
