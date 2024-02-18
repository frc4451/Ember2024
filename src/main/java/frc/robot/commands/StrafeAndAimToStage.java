package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.apriltag.StageTags;

public class StrafeAndAimToStage extends StrafeAndAimToAprilTag {
    /**
     * @param idOffset 0-2 for the speaker tags
     */
    public StrafeAndAimToStage(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            StageTags stageTag,
            DriveSubsystem drive) {
        super(xSupplier,
                ySupplier,
                visibleAprilTagsSupplier,
                stageTag.getId(),
                drive,
                false);
        setName("StrafeAndAimToStage");
    }
}
