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

        // Angle collected from spreadsheet, distance from tag calculated from
        // PathPlanner. Update this if this is not correct.
        armAngleMap.put(1.35, 55.0);
        // These are straight from the data collected
        armAngleMap.put(Units.feetToMeters(10), PivotLocation.k36.angle.getDegrees());
        armAngleMap.put(Units.feetToMeters(13), 31.0);
        armAngleMap.put(Units.feetToMeters(15), PivotLocation.k26.angle.getDegrees());
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

        double distanceToTarget = GeomUtils.getDistanceFromTag(this.drivePoseSupplier.get(), StageTags.SPEAKER_AIM);
        Logger.recordOutput("Pivot/EstimatedDistanceToTarget", distanceToTarget);
        Logger.recordOutput("Pivot/EstimatedNeededAngle", armAngleMap.get(distanceToTarget));
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
