package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.GarageUtils;

public class StrafeAndAimToSpeaker extends StrafeAndAimToAprilTag {
    public StrafeAndAimToSpeaker(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DriveSubsystem drive) {
        super(xSupplier,
                ySupplier,
                GarageUtils.isBlueAlliance()
                        ? VisionConstants.BLUE_SPEAKER_CENTER
                        : VisionConstants.RED_SPEAKER_CENTER,
                drive);
        setName("StrafeAndAimToSpeaker");
    }
}
