package frc.utils.TargetAngleTrackers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public abstract class TargetAngleTracker {
    protected final PIDController thetaController;
    protected final double offset;

    public TargetAngleTracker(
            double kP) {
        this(kP, 0.0, 0.0, 0.0, false);
    }

    public TargetAngleTracker(
            double kP,
            double kI,
            double kD) {
        this(kP, kI, kD, 0.0, false);
    }

    public TargetAngleTracker(
            double kP,
            double kI,
            double kD,
            double offset) {
        this(kP, kI, kD, offset, false);
    }

    public TargetAngleTracker(
            double kP,
            double kI,
            double kD,
            double offset,
            boolean isContinuous) {
        this.thetaController = new PIDController(kP, kI, kD);

        this.offset = offset;

        this.thetaController.setTolerance(Units.degreesToRadians(0.1));

        if (isContinuous) {
            this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }
    }

    public abstract void update();

    public abstract Rotation2d getRotationDifference();

    public abstract void log();
}
