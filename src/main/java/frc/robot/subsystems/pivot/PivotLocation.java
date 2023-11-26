package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotLocation {
    INITIAL(Rotation2d.fromDegrees(184.0)),
    k0(Rotation2d.fromDegrees(0.0)),
    k90(Rotation2d.fromDegrees(90.0)),
    k160(Rotation2d.fromDegrees(160.0)),
    k167(Rotation2d.fromDegrees(167.25));

    public Rotation2d angle;

    PivotLocation(Rotation2d angle) {
        this.angle = angle;
    }
}
