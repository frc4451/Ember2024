package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotLocation {
    INITIAL(Rotation2d.fromDegrees(26.0)),
    k36(Rotation2d.fromDegrees(36)),
    k45(Rotation2d.fromDegrees(45.0)),
    kSoftMin(Rotation2d.fromDegrees(28.0)),
    kElevatorDownSoftMax(Rotation2d.fromDegrees(40.0)),
    kElevatorDownHardMax(Rotation2d.fromDegrees(42.0)),
    kAmpScoringPosition(Rotation2d.fromDegrees(63.0)), // fill this in later
    kTrapScoringPosition(Rotation2d.fromDegrees(42.0)), // fill this in later
    kElevatorUpSoftMax(Rotation2d.fromDegrees(82.0)),
    ;

    public Rotation2d angle;

    PivotLocation(Rotation2d angle) {
        this.angle = angle;
    }
}
