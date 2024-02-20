package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.subsystems.vision.apriltag.StageTags;
import frc.utils.GeomUtils;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private Rotation2d angle = new Rotation2d();
    private Rotation2d setpoint = new Rotation2d();

    private final PIDController pidController = new PIDController(
            Constants.IntakeConstants.kPivotP,
            Constants.IntakeConstants.kPivotI,
            Constants.IntakeConstants.kPivotD);

    private PivotAimingParameters pivotAimingParameters = null;
    private final Supplier<Pose2d> drivePoseSupplier;

    /**
     * <p>
     * Lookup table for finding known good shots and guessing the angle we need
     * between those shots.
     * </p>
     * <p>
     * key: meters, values: degrees
     * </p>
     */
    private final InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();

    // Mechanisms
    private final PivotVisualizer measuredVisualizer = new PivotVisualizer("measured", Color.kBlack);
    private final PivotVisualizer setpointVisualizer = new PivotVisualizer("setpoint", Color.kGreen);
    private final PivotVisualizer goalVisualizer = new PivotVisualizer("goal", Color.kBlue);

    public PivotSubsystem(Supplier<Pose2d> drivePoseSupplier) {
        switch (AdvantageKitConstants.getMode()) {
            case REAL:
                io = new PivotIOSparkMax();
                break;
            case SIM:
                io = new PivotIOSim();
                break;
            case REPLAY:
            default:
                io = new PivotIO() {
                };
                break;
        }

        setAngle(PivotLocation.INITIAL.angle);
        setSetpoint(PivotLocation.INITIAL.angle);

        this.drivePoseSupplier = drivePoseSupplier;

        // Default Angle if it's in-line with the speaker tag
        armAngleMap.put(0.0, 55.0);

        // Angle collected from spreadsheet, distance from tag calculated from
        // PathPlanner. Update this if this is not correct.
        armAngleMap.put(1.35, 55.0);
        // These are straight from the data collected
        armAngleMap.put(Units.feetToMeters(10), PivotLocation.k36.angle.getDegrees());
        armAngleMap.put(Units.feetToMeters(13), 31.0);
        armAngleMap.put(Units.feetToMeters(15), PivotLocation.k26.angle.getDegrees());

        // The "hard limit" of where we want our pivot moving.
        armAngleMap.put(Double.MAX_VALUE, 25.0);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Pivot", this.inputs);

        // Make sure the motor actually stops when the robot disabled
        if (DriverStation.isDisabled()) {
            this.setSetpoint(PivotLocation.INITIAL.angle);
            this.io.stop();
        }

        this.angle = new Rotation2d(this.inputs.relativeAngleRad);

        Logger.recordOutput("Pivot/Angle", getAngle().getDegrees());
        Logger.recordOutput("Pivot/SetpointAngle", getSetpoint().getDegrees());

        // Log Aimed Pivot shooter
        double distanceToTarget = getDistanceFromTag();
        double estimatedNeededAngleDeg = armAngleMap.get(distanceToTarget);
        Logger.recordOutput("Pivot/EstimatedDistanceToTarget", distanceToTarget);
        Logger.recordOutput("Pivot/EstimatedNeededAngle", estimatedNeededAngleDeg);

        // Log Mechanisms - This needs to be recorded in Radians
        measuredVisualizer.update(getAngle().getRadians());
        setpointVisualizer.update(getSetpoint().getRadians());
        goalVisualizer.update(Rotation2d.fromDegrees(estimatedNeededAngleDeg).getRadians());
    }

    public void setAngle(Rotation2d angle) {
        this.io.setAngle(angle);
    }

    public Rotation2d getAngle() {
        return this.angle;
    }

    public Rotation2d getSetpoint() {
        return this.setpoint;
    }

    private double getDistanceFromTag() {
        return GeomUtils.getDistanceFromTag(this.drivePoseSupplier.get(), StageTags.SPEAKER_AIM);
    }

    private void setSetpoint(Rotation2d angle) {
        this.setpoint = angle;
        this.pidController.setSetpoint(angle.getDegrees());
    }

    public Command setSetpointCommand(Rotation2d angle) {
        return new InstantCommand(() -> this.setSetpoint(angle));
    }

    public Command setSetpointCurrentCommand() {
        return new InstantCommand(() -> this.setSetpoint(this.angle));
    }

    public Command pivotPIDCommand() {
        return new RunCommand(() -> {
            double output = this.pidController.calculate(this.getAngle().getDegrees());
            useOutput(output);
        }, this);
    }

    public void useOutput(double output) {
        this.io.setVoltage(MathUtil.clamp(output, -12, 12));
    }

    public Command runPercentCommand(DoubleSupplier decimalPercent) {
        return new RunCommand(() -> this.io.setPercentOutput(decimalPercent.getAsDouble()), this);
    }

    public PivotAimingParameters getPivotAimingParameters() {
        if (pivotAimingParameters != null) {
            return pivotAimingParameters;
        }

        return pivotAimingParameters;
    }
}
