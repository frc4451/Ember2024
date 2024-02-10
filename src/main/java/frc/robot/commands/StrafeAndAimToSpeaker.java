package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.utils.GarageUtils;

public class StrafeAndAimToSpeaker extends StrafeAndAimToAprilTag {
    public StrafeAndAimToSpeaker(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Set<TargetWithSource>> visibleAprilTagsSupplier,
            DriveSubsystem drive) {
        super(xSupplier,
                ySupplier,
                visibleAprilTagsSupplier,
                GarageUtils.isBlueAlliance()
                        ? VisionConstants.BLUE_SPEAKER_CENTER
                        : VisionConstants.RED_SPEAKER_CENTER,
                drive);
        setName("StrafeAndAimToSpeaker");
    }
}
