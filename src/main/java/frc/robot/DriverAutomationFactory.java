package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PositionWithAmp;
import frc.robot.commands.PositionWithStageSingleClimb;
import frc.robot.commands.StrafeAndAimToPose;
import frc.robot.commands.StrafeAndAimToSpeaker;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.pivot.PivotLocation;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.apriltag.OffsetTags;
import frc.utils.CommandCustomController;
import frc.utils.GarageUtils;

public class DriverAutomationFactory {
    private final CommandCustomController driverController;
    // private final CommandCustomController operatorController;

    private final DriveSubsystem drive;
    private final PivotSubsystem pivot;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public DriverAutomationFactory(
            CommandCustomController driverController,
            CommandCustomController operatorController,
            DriveSubsystem drive,
            PivotSubsystem pivot,
            ShooterSubsystem shooter,
            FeederSubsystem feeder) {
        this.driverController = driverController;
        // this.operatorController = operatorController;
        this.drive = drive;
        this.pivot = pivot;
        this.shooter = shooter;
        this.feeder = feeder;
    }

    public Command aimAtSpeakerAssist() {
        return Commands.defer(
                () -> new StrafeAndAimToSpeaker(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        drive),
                Set.of(drive));
    }

    public Command ampPath() {
        return OffsetTags.AMP.getDeferredCommand();
    }

    public Command ampAssist() {
        return Commands.defer(() -> new PositionWithAmp(
                () -> -driverController.getLeftX(),
                drive,
                OffsetTags.AMP),
                Set.of(drive));
    }

    public Command stageHumanPath() {
        return OffsetTags.STAGE_HUMAN.getDeferredCommand();
    }

    public Command stageHumanAssist() {
        return Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> driverController.getLeftY(),
                OffsetTags.STAGE_HUMAN,
                drive),
                Set.of(drive));
    }

    public Command stageAmpPath() {
        return OffsetTags.STAGE_AMP.getDeferredCommand();
    }

    public Command stageAmpAssist() {
        return Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> driverController.getLeftY(),
                OffsetTags.STAGE_AMP,
                drive),
                Set.of(drive));
    }

    public Command stageCenterPath() {
        return OffsetTags.STAGE_CENTER.getDeferredCommand();
    }

    public Command stageCenterAssist() {
        return Commands.defer(() -> new PositionWithStageSingleClimb(
                () -> -driverController.getLeftY(),
                OffsetTags.STAGE_CENTER,
                drive),
                Set.of(drive));
    }

    public Command stageLeftPath() {
        return Commands.deferredProxy(() -> GarageUtils.isBlueAlliance()
                ? OffsetTags.STAGE_AMP.getDeferredCommand()
                : OffsetTags.STAGE_HUMAN.getDeferredCommand());
    }

    public Command stageLeftAssist() {
        return Commands.deferredProxy(() -> GarageUtils.isBlueAlliance() ? stageAmpAssist() : stageHumanAssist());
    }

    public Command stageRightPath() {
        return Commands.deferredProxy(() -> GarageUtils.isBlueAlliance()
                ? OffsetTags.STAGE_HUMAN.getDeferredCommand()
                : OffsetTags.STAGE_AMP.getDeferredCommand());
    }

    public Command stageRightAssist() {
        return Commands.deferredProxy(() -> GarageUtils.isBlueAlliance()
                ? stageHumanAssist()
                : stageAmpAssist());
    }

    public Command humanPlayerStationPath() {
        return Commands.deferredProxy(() -> OffsetTags.HUMAN_PLAYER.getDeferredCommand());
    }

    public Command ampShot() {
        Rotation2d shootAngle = PivotLocation.kNewAmpShot.angle.minus(Rotation2d.fromDegrees(10));
        Rotation2d finalAngle = PivotLocation.kNewAmpShot.angle;

        return Commands.sequence(
                pivot.setGoalCommand(finalAngle),
                shooter.shootIntoAmpCommand(),
                Commands.waitUntil(pivot.isNearAngle(shootAngle)),
                feeder.feedIntoAmpCommand(),
                Commands.waitSeconds(0.25),
                shooter.stopCommand(),
                feeder.stopCommand());
    }

    public Command strafeAndAimToAmpFeed() {
        return new StrafeAndAimToPose(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                OffsetTags.FLOOR_SHOT::getOffsetPose,
                drive,
                true);

    }
}
