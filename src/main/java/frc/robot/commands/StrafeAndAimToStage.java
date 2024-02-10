package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.utils.GarageUtils;

public class StrafeAndAimToStage extends StrafeAndAimToAprilTag {
    /**
     * @param idOffset 0-2 for the speaker tags
     */
    public StrafeAndAimToStage(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            DriveSubsystem drive,
            int idOffset) {
        super(xSupplier,
                ySupplier,
                visibleAprilTagsSupplier,
                (GarageUtils.isBlueAlliance() ? 14 : 11) + idOffset,
                drive,
                false);
        setName("StrafeAndAimToStage");
    }
}
